#include "quad_motor_control/quad_motor_control.hpp"

// Control table address for X series (except XL-320)
#define ADDR_OPERATING_MODE 11
#define ADDR_TORQUE_ENABLE 64
#define ADDR_GOAL_POSITION 116
#define ADDR_PRESENT_POSITION 132

// Protocol version
#define PROTOCOL_VERSION 2.0  // Default Protocol version of DYNAMIXEL X series.

// Data Byte Length
#define LEN_PRESENT_POSITION            4

// Default setting
#define BAUDRATE 57600  // Default Baudrate of DYNAMIXEL X series
#define DEVICE_NAME "/dev/ttyUSB0"  // [Linux]: "/dev/ttyUSB*", [Windows]: "COM*"

#define NUM_MOTORS 12
#define MOTOR_READ_FAIL -1

// Includes for I2C
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <cmath>

uint8_t dxl_error = 0;
uint32_t goal_position = 0;
int dxl_comm_result = COMM_TX_FAIL;


QuadMotorControl::QuadMotorControl() : Node("quad_motor_control"), last_executed_config_(-1), curr_robot_state_(RobotStateEnum::TURNING) 
{
    RCLCPP_INFO(this->get_logger(), "Run quad_motor_control node");

    this->declare_parameter("qos_depth", 10);
    int8_t qos_depth = 0;
    this->get_parameter("qos_depth", qos_depth);

    const auto QOS_RKL10V =
        rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

    this->portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
    this->packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
    // Initialize GroupSyncWrite instance
    this->groupSyncWrite = new dynamixel::GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_PRESENT_POSITION);
    // Initialize GroupsyncRead instance for Present Position
    // this->groupSyncRead = new dynamixel::GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);

    this->initDynamixels();

    // Initialize IMU
    initIMU();


    set_position_subscriber_ =
        this->create_subscription<SetPosition>(
        "set_position", // topic
        QOS_RKL10V,
        [this](const SetPosition::SharedPtr msg) -> void
        {
            uint8_t dxl_error = 0;

            // Position Value of X series is 4 byte data.
            // For AX & MX(1.0) use 2 byte data(uint16_t) for the Position Value.
            uint32_t goal_position = (unsigned int)msg->position;  // Convert int32 -> uint32

            // Write Goal Position (length : 4 bytes)
            // When writing 2 byte data to AX / MX(1.0), use write2ByteTxRx() instead.
            dxl_comm_result =
            packetHandler->write4ByteTxRx(
                portHandler,
                (uint8_t) msg->id,
                ADDR_GOAL_POSITION,
                goal_position,
                &dxl_error
            );

            if (dxl_comm_result != COMM_SUCCESS) {
                RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getTxRxResult(dxl_comm_result));
            } else if (dxl_error != 0) {
                RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getRxPacketError(dxl_error));
            } else {
                RCLCPP_INFO(this->get_logger(), "Set [ID: %d] [Goal Position: %d]", msg->id, msg->position);
            }
        }
        );

    robot_state_subscriber_ = this->create_subscription<quad_interfaces::msg::RobotState>(
        "/robot_state",
        rclcpp::QoS(1).best_effort().durability_volatile(),
        [this](const quad_interfaces::msg::RobotState::SharedPtr msg) -> void {
            curr_robot_state_ = static_cast<RobotStateEnum>(msg->current_state);
            RCLCPP_INFO(this->get_logger(), "Robot state updated: %d", static_cast<int>(curr_robot_state_));
        }
    );


    const auto QOS_KEEP_LATEST = rclcpp::QoS(rclcpp::KeepLast(1))
        .best_effort()  // Do NOT queue old messages, discard outdated ones
        .durability_volatile();  // Do NOT persist messages
        
    set_config_subscriber_ =
        this->create_subscription<SetConfig>(
        "set_config", // topic
        QOS_KEEP_LATEST,  // Set QoS depth to 1
        [this](const SetConfig::SharedPtr msg) -> void
        {   
            int config_id = msg->config_id;
            
            // Allow turning only if the robot is still in a state before STOPPED_TURNING
            if (config_id == 5 && curr_robot_state_ < RobotStateEnum::STOPPED_TURNING) {
                RCLCPP_INFO(this->get_logger(), "Robot is still turning, executing config 5.");
                execute_config(5);
            } else if (config_id == 5 && curr_robot_state_ >= RobotStateEnum::STOPPED_TURNING) {
                RCLCPP_WARN(this->get_logger(), "Ignoring redundant config 5, robot has already stopped turning.");
                return;
            }


            last_executed_config_ = config_id;  // Store latest config

            RCLCPP_INFO(this->get_logger(), "🔥 Processing latest config update: %d", config_id);

            // Clear any previous queued SyncWrite commands
            groupSyncWrite->clearParam();

            // Execute immediate transformation
            execute_config(config_id);
        }
    );


    auto get_present_position =
        [this](
        const std::shared_ptr<GetPosition::Request> request,
        std::shared_ptr<GetPosition::Response> response) -> void
        {
        // Read Present Position (length : 4 bytes) and Convert uint32 -> int32
        // When reading 2 byte data from AX / MX(1.0), use read2ByteTxRx() instead.
        dxl_comm_result = packetHandler->read4ByteTxRx(
            portHandler,
            (uint8_t) request->id,
            ADDR_PRESENT_POSITION,
            reinterpret_cast<uint32_t *>(&present_position),
            &dxl_error
        );

        RCLCPP_INFO(
            this->get_logger(),
            "Get [ID: %d] [Present Position: %d]",
            request->id,
            present_position
        );

        response->position = present_position;
        };

    get_position_server_ = create_service<GetPosition>("get_position", get_present_position);


    auto get_all_id_positions =
        [this](
        [[maybe_unused]] 
        const std::shared_ptr<GetAllPositions::Request> request,
        std::shared_ptr<GetAllPositions::Response> response) -> void
        {
            for (int id = 1; id <= NUM_MOTORS; id++) {
                uint32_t motor_position = 0;
                // Read Present Position (length : 4 bytes) and Convert uint32 -> int32
                // When reading 2 byte data from AX / MX(1.0), use read2ByteTxRx() instead.
                dxl_comm_result = packetHandler->read4ByteTxRx(
                    portHandler,
                    (uint8_t) id,
                    ADDR_PRESENT_POSITION,
                    &motor_position,
                    &dxl_error
                );

                RCLCPP_INFO(
                    this->get_logger(),
                    "Get [ID: %d] [Present Position: %d]",
                    id,
                    motor_position
                );

                switch (id) {
                case 1: response->motor1_position = motor_position; break;
                case 2: response->motor2_position = motor_position; break;
                case 3: response->motor3_position = motor_position; break;
                case 4: response->motor4_position = motor_position; break;
                case 5: response->motor5_position = motor_position; break;
                case 6: response->motor6_position = motor_position; break;
                case 7: response->motor7_position = motor_position; break;
                case 8: response->motor8_position = motor_position; break;
                case 9: response->motor9_position = motor_position; break;
                case 10: response->motor10_position = motor_position; break;
                case 11: response->motor11_position = motor_position; break;
                case 12: response->motor12_position = motor_position; break;
                }
            }
        };

    get_all_positions_server_ = create_service<GetAllPositions>("get_all_positions", get_all_id_positions);
    motor_positions_publisher_ = this->create_publisher<quad_interfaces::msg::MotorPositions>("/motor_positions", 10);

    auto timer_callback =
      [this]() -> void {
        auto message = quad_interfaces::msg::MotorPositions();

        // Read motor positions
        for (int id = 1; id <= NUM_MOTORS; id++) {
            uint32_t motor_position = 0;
            dxl_comm_result = packetHandler->read4ByteTxRx(
                portHandler,
                (uint8_t) id,
                ADDR_PRESENT_POSITION,
                &motor_position,
                &dxl_error
            );

            // Assign to message
            switch (id) {
                case 1: message.motor1_position = motor_position; break;
                case 2: message.motor2_position = motor_position; break;
                case 3: message.motor3_position = motor_position; break;
                case 4: message.motor4_position = motor_position; break;
                case 5: message.motor5_position = motor_position; break;
                case 6: message.motor6_position = motor_position; break;
                case 7: message.motor7_position = motor_position; break;
                case 8: message.motor8_position = motor_position; break;
                case 9: message.motor9_position = motor_position; break;
                case 10: message.motor10_position = motor_position; break;
                case 11: message.motor11_position = motor_position; break;
                case 12: message.motor12_position = motor_position; break;
            }
        }

        this->motor_positions_publisher_->publish(message);

        // Check tilt angle if IMU is working
        if (i2c_file > 0) {
            float tilt_angle = getTiltAngle();
            
            // Only process valid readings
            if (tilt_angle != -1000.0f) {
                RCLCPP_DEBUG(this->get_logger(), "Current tilt angle: %.2f degrees", tilt_angle);
                
                // Determine orientation based on tilt angle
                bool blue_under = (tilt_angle >= -180 && tilt_angle <= -122) ||
                                (tilt_angle >= 123 && tilt_angle <= 180);
                bool yellow_under = tilt_angle >= -54 && tilt_angle <= 58;
                
                // If in ROLLING state and not already moving
                if (curr_robot_state_ >= RobotStateEnum::ROLLING) {
                    // Example of how to trigger a roll based on orientation
                    if (yellow_under) {
                        RCLCPP_INFO(this->get_logger(), "Yellow side under, initiating yellow push");
                        // Call yellow push sequence
                        // For example:
                        execute_roll_yellow();
                    } else if (blue_under) {
                        RCLCPP_INFO(this->get_logger(), "Blue side under, initiating blue push");
                        // Call blue push sequence
                        // For example:
                        execute_roll_blue();
                    }
                }
            }
        }
      };
    timer_ = this->create_wall_timer(std::chrono::milliseconds(500), timer_callback);
}

QuadMotorControl::~QuadMotorControl()
{
    if (i2c_file > 0) {
        close(i2c_file);
    }
}

void QuadMotorControl::execute_roll_yellow() {
    gradual_transition(yellow_up_cir);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    gradual_transition(perfect_cir);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
}

void QuadMotorControl::execute_roll_blue() {
    gradual_transition(blue_up_cir);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    gradual_transition(perfect_cir);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
}

void QuadMotorControl::execute_config(int config_id) {
    std::vector<int*> config_sequence;
    std::vector<int> sleep_durations;

    // Your existing switch statement remains the same
    switch (config_id) {
        case 1: 
            config_sequence = {home_tiptoe};
            sleep_durations = {0}; 
            break;
        case 2: 
            config_sequence = {perfect_cir};
            sleep_durations = {0};  
            break;
        case 3:                             // Home to Cir
            config_sequence = {aligned_before_rolling, walk_to_cir1, perfect_cir};
            sleep_durations = {500, 300, 0}; // Time delays between steps
            break;
        case 4:                             // Cir to Home
            config_sequence = {perfect_cir, cir_to_blue3_180, cir_to_both_blues_180, cir_to_yellow_up60, cir_to_yellow_up90, aligned_before_rolling, home_tiptoe_thin, home_tiptoe};
            sleep_durations = {700, 1000, 1000, 1000, 1000, 1000, 1000}; // Time delays between steps
            break;
        case 5:  // Turning Right Sequence
            config_sequence = {
                leg4_up_right, leg4_turn_right, leg4_down_right,
                leg3_up_right, leg3_turn_right, leg3_down_right, 
                leg2_up_right, leg2_turn_right, leg2_down_right,
                leg1_up_right, leg1_turn_right, leg1_down_right,
                home_tiptoe
            };
            sleep_durations = {500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 7000}; // 500ms delay per step
            // sleep_durations = {100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100};
            break;
    }
    
    // Use gradual transition for specific transitions
    if (config_id == 3) {  // Home to Cir - use gradual transition
        // First get to aligned position
        gradual_transition(aligned_before_rolling);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        // Then to walk_to_cir1
        gradual_transition(walk_to_cir1);
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        
        // Finally to perfect_cir
        gradual_transition(perfect_cir);
    }
    else if (config_id == 4) {  // Cir to Home - use gradual transition
        // Process each step with gradual transition
        gradual_transition(perfect_cir);
        std::this_thread::sleep_for(std::chrono::milliseconds(700));
        
        gradual_transition(cir_to_blue3_180);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        
        // Continue with each step...
        gradual_transition(cir_to_both_blues_180);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        
        gradual_transition(cir_to_yellow_up60);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        
        gradual_transition(cir_to_yellow_up90);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        
        gradual_transition(aligned_before_rolling);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        
        gradual_transition(home_tiptoe_thin);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        
        gradual_transition(home_tiptoe);
    }
    else {
        // For other configs, use the original method
        for (size_t i = 0; i < config_sequence.size(); i++) {
            apply_motor_positions(config_sequence[i]);
            
            if (i < sleep_durations.size()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(sleep_durations[i]));
            }
        }
    }
}

void QuadMotorControl::apply_motor_positions(int* target_positions) {
    groupSyncWrite->clearParam();
    
    for (int id = 1; id <= NUM_MOTORS; id++) {
        uint8_t param_goal_position[4];
        int goal_position = target_positions[id];

        param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(goal_position));
        param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(goal_position));
        param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(goal_position));
        param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(goal_position));

        if (!groupSyncWrite->addParam(id, param_goal_position)) {
            RCLCPP_WARN(this->get_logger(), "[ID:%03d] SyncWrite addParam failed", id);
        }
    }

    // **Transmit transformation immediately**
    int dxl_comm_result = groupSyncWrite->txPacket();
    if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "SyncWrite Failed: %s", packetHandler->getTxRxResult(dxl_comm_result));
    }

    groupSyncWrite->clearParam(); // Clear after sending
}


void QuadMotorControl::initDynamixels()
{
    // Open Serial Port
    dxl_comm_result = this->portHandler->openPort();
    if (dxl_comm_result == false) {
        RCLCPP_ERROR(rclcpp::get_logger("read_write_node"), "Failed to open the port!");
    } else {
        RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "Succeeded to open the port.");
    }

    // Set the baudrate of the serial port (use DYNAMIXEL Baudrate)
    dxl_comm_result = portHandler->setBaudRate(BAUDRATE);
    if (dxl_comm_result == false) {
        RCLCPP_ERROR(rclcpp::get_logger("read_write_node"), "Failed to set the baudrate!");
    } else {
        RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "Succeeded to set the baudrate.");
    }

    // Use Position Control Mode
    dxl_comm_result = packetHandler->write1ByteTxRx(
        this->portHandler,
        BROADCAST_ID,
        ADDR_OPERATING_MODE,
        3,
        &dxl_error
    );

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("quad_motor_control"), "Failed to set Position Control Mode.");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("quad_motor_control"), "Succeeded to set Position Control Mode.");
  }

  // Enable Torque of DYNAMIXEL
  dxl_comm_result = packetHandler->write1ByteTxRx(
    this->portHandler,
    BROADCAST_ID,
    ADDR_TORQUE_ENABLE,
    1,
    &dxl_error
  );

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("quad_motor_control"), "Failed to enable torque.");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("quad_motor_control"), "Succeeded to enable torque.");
  }
}

void QuadMotorControl::initIMU() {
    // Open I2C device
    int adapter_nr = 1;  // Use /dev/i2c-1
    char filename[20];
    snprintf(filename, 19, "/dev/i2c-%d", adapter_nr);
    i2c_file = open(filename, O_RDWR);
    if (i2c_file < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open I2C bus");
        return;
    }

    // Set I2C device address (LSM330DHCX)
    int addr = 0x6A;
    if (ioctl(i2c_file, I2C_SLAVE, addr) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set I2C address");
        close(i2c_file);
        i2c_file = -1;
        return;
    }

    // Enable gyroscope and accelerometer
    write_register(0x10, 0x60);  // Accelerometer
    write_register(0x11, 0x60);  // Gyroscope
    
    // Initialize IMU variables
    accumulated_tilt_angle = 0.0f;
    sample_count = 0;
    
    RCLCPP_INFO(this->get_logger(), "IMU initialized successfully");
    
    // Sleep to allow sensor to initialize
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}

int QuadMotorControl::write_register(uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {reg, value};
    if (write(i2c_file, buf, 2) != 2) {
        RCLCPP_ERROR(this->get_logger(), "Failed to write register 0x%x", reg);
        return -1;
    }
    return 0;
}

int QuadMotorControl::read_register(uint8_t reg) {
    uint8_t data;
    if (write(i2c_file, &reg, 1) != 1) {
        RCLCPP_ERROR(this->get_logger(), "Failed to write register address: 0x%x", reg);
        return -1;
    }
    if (read(i2c_file, &data, 1) != 1) {
        RCLCPP_ERROR(this->get_logger(), "Failed to read from register: 0x%x", reg);
        return -1;
    }
    return data;
}

int16_t QuadMotorControl::read_16bit_register(uint8_t reg_low, uint8_t reg_high) {
    int16_t low = read_register(reg_low);
    int16_t high = read_register(reg_high);
    if (low == -1 || high == -1) return -1;
    return (high << 8) | low;
}

float QuadMotorControl::getTiltAngle() {
    // Read accelerometer data
    // int16_t accel_x = read_16bit_register(0x28, 0x29);
    int16_t accel_y = read_16bit_register(0x2A, 0x2B);
    int16_t accel_z = read_16bit_register(0x2C, 0x2D);
    
    // Apply scale factors and bias correction
    float accel_z_offset = 0.2;
    // float accel_mps2_x = accel_x * (2.0 / 32768.0) * 9.81;
    float accel_mps2_y = accel_y * (2.0 / 32768.0) * 9.81;
    float accel_mps2_z = ((accel_z * (2.0 / 32768.0)) * 9.81) - accel_z_offset;
    
    // Compute tilt angle around x-axis
    float angle_rad = std::atan2(accel_mps2_y, accel_mps2_z);
    float angle_degrees = angle_rad * (180.0 / M_PI);
    
    // Add to accumulated values for averaging
    accumulated_tilt_angle += angle_degrees;
    sample_count++;
    
    // If we have enough samples, return the average
    if (sample_count >= window_size) {
        float avg_tilt_angle = accumulated_tilt_angle / sample_count;
        
        // Reset for next reading
        accumulated_tilt_angle = 0;
        sample_count = 0;
        
        return avg_tilt_angle;
    }
    
    // Not enough samples yet
    return -1000.0f;  // Special value indicating not ready
}

void QuadMotorControl::update_present_positions() {
    for (int id = 1; id <= NUM_MOTORS; id++) {
        uint32_t motor_position = 0;
        dxl_comm_result = packetHandler->read4ByteTxRx(
            portHandler,
            (uint8_t) id,
            ADDR_PRESENT_POSITION,
            &motor_position,
            &dxl_error
        );
        
        if (dxl_comm_result == COMM_SUCCESS) {
            present_positions[id] = motor_position;
        }
    }
}

void QuadMotorControl::gradual_transition(int* next_positions) {
    const int step_size = 17;
    float step_arr[NUM_MOTORS + 1] = {0};
    int num_motors = NUM_MOTORS;

    int updated_positions[NUM_MOTORS + 1] = {0};

    // Ensure present_positions is updated before starting the transition
    update_present_positions();
    
    std::copy(std::begin(present_positions), std::end(present_positions), std::begin(updated_positions));

    for (int i = 1; i <= num_motors; i++) {
        step_arr[i] = static_cast<float>(next_positions[i] - updated_positions[i]) / step_size;
    }

    for (int step = 0; step < step_size; step++) {
        for (int i = 1; i <= num_motors; i++) {
            updated_positions[i] += std::round(step_arr[i]);  // Fix rounding issue
        }
        apply_motor_positions(updated_positions);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    // Ensure we reach the exact target position
    apply_motor_positions(next_positions);
}

int main(int argc, char * argv[]) {
    initialize_turning_configs_right();  // Ensure all arrays are set up
    initialize_relative_configs();
    initialize_rolling_configs();
    
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<QuadMotorControl>());
    rclcpp::shutdown();
    return 0;
}
