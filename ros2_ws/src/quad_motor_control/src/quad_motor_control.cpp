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


uint8_t dxl_error = 0;
uint32_t goal_position = 0;
int dxl_comm_result = COMM_TX_FAIL;


QuadMotorControl::QuadMotorControl() : Node("quad_motor_control")
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
    
    set_config_subscriber_ =
        this->create_subscription<SetConfig>(
        "set_config", // topic
        QOS_RKL10V,
        [this](const SetConfig::SharedPtr msg) -> void
        {   
            int config_id = msg->config_id;
            int* target_positions = home_tiptoe;

            switch (config_id) {
                case 1: target_positions = home_tiptoe; break;
                case 2: target_positions = perfect_cir; break;
            }
            // Clear previous SyncWrite parameters
            groupSyncWrite->clearParam();
            
            for (int id = 1; id <= NUM_MOTORS; id++)  // Loop through motor IDs 1-12
            {
                uint8_t param_goal_position[4];
                int goal_position = target_positions[id];

                param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(goal_position));
                param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(goal_position));
                param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(goal_position));
                param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(goal_position));

                // Add goal position to SyncWrite buffer
                if (!groupSyncWrite->addParam(id, param_goal_position)) {
                    fprintf(stderr, "[ID:%03d] groupSyncWrite addParam failed\n", id);
                    continue;
                }
            }

            // Transmit the target positions to all motors at once
            int dxl_comm_result = groupSyncWrite->txPacket();
            if (dxl_comm_result != COMM_SUCCESS) {
                printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
            }

            // Clear SyncWrite buffer after sending data
            groupSyncWrite->clearParam();
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

}

QuadMotorControl::~QuadMotorControl()
{
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

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<QuadMotorControl>());
    rclcpp::shutdown();
    return 0;
}
