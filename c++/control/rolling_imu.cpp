// === Standard C/C++ Libraries ===
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <unordered_map>
#include <random>
#include <thread>
#include <chrono>

// === System Headers ===
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>

#if defined(__linux__) || defined(__APPLE__)
    #include <termios.h>
    #define STDIN_FILENO 0
#elif defined(_WIN32) || defined(_WIN64)
    #include <conio.h>
#endif

// === Linux I2C ===
#include <linux/i2c-dev.h>

// === Dynamixel SDK ===
#include "dynamixel_sdk.h"  // Uses Dynamixel SDK library

// === Macro Definitions ===

// Control table addresses
#define ADDR_PRO_TORQUE_ENABLE          64
#define ADDR_PRO_GOAL_POSITION          116
#define ADDR_PRESENT_POSITION           132

// Data Byte Length
#define LEN_PRO_GOAL_POSITION           4
#define LEN_PRESENT_POSITION            4

// Protocol version
#define PROTOCOL_VERSION                2.0

// Default settings
#define BAUDRATE                        57600
#define DEVICENAME                      "/dev/ttyUSB0"

#define TORQUE_ENABLE                   1
#define TORQUE_DISABLE                  0
#define DXL_MINIMUM_POSITION_VALUE      0
#define DXL_MAXIMUM_POSITION_VALUE      4095
#define DXL_MOVING_STATUS_THRESHOLD     20

#define ESC_ASCII_VALUE                 0x1b
#define NUM_MOTORS                      12
#define MAX_INPUT_SIZE                  100

// === Utility Functions ===

int write_register(int file, uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {reg, value};
    if (write(file, buf, 2) != 2) {
        std::cerr << "Failed to write register 0x" << std::hex << (int)reg << std::endl;
        return -1;
    }
    return 0;
}

int read_register(int file, uint8_t reg) {
    uint8_t data;
    if (write(file, &reg, 1) != 1) {
        std::cerr << "Failed to write register address: 0x" << std::hex << (int)reg << std::endl;
        return -1;
    }
    if (read(file, &data, 1) != 1) {
        std::cerr << "Failed to read from register: 0x" << std::hex << (int)reg << std::endl;
        return -1;
    }
    return data;
}

int16_t read_16bit_register(int file, uint8_t reg_low, uint8_t reg_high) {
    int16_t low = read_register(file, reg_low);
    int16_t high = read_register(file, reg_high);
    if (low == -1 || high == -1) return -1;
    return (high << 8) | low;
}


int DXL_ID;
bool toggle_position = false;  // Toggles between the two positions
bool forward_running = false;

// ALL CONFIGS
// 1 for WALKING, 2 for ROLLING
int mode = 1;
int present_positions[NUM_MOTORS + 1] = {0, 
  2045, 2053, 3049, 2054, 2035, 1014, 2044, 2047, 3071, 3051, 2043, 1056
};

///////////////////////////////// ROLLING START /////////////////////////////////
const double TICKS_PER_DEGREE = 4096.0 / 360.0;

const int UP_DOWN_TICKS_ROLL = static_cast<int>(50 * TICKS_PER_DEGREE); 
const int UP_TICKS_PROPEL_SMALL = static_cast<int>(30 * TICKS_PER_DEGREE);

// Circle Positions
int perfect_cir[NUM_MOTORS + 1] = {0, 
  2039, 1113, 3080, 2053, 2980, 1006, 2086, 2983, 1045, 3054, 1112, 3094
};

// BLUE
// Leg 3 and 4 Movements
int blue_up_propel[NUM_MOTORS + 1] = {0}; 

// YELLOW
// Leg 1 and 2 Movements
int yellow_up_propel[NUM_MOTORS + 1] = {0};

///////////////////////////////// ROLLING END /////////////////////////////////

// Function to copy array
void copy_array(int* dest, int* src) {
    for (int i = 0; i <= NUM_MOTORS; i++) {
        dest[i] = src[i];
    }
}

// Function to populate movement arrays based on sequence
void generate_movement_arrays_roll_fw() {
  // Start with perfect_cir for all movements
  copy_array(blue_up_propel, perfect_cir);
  copy_array(yellow_up_propel, perfect_cir);
  
  std::cout << "GENERATING FOR BLUE LEGS\n";
  // BLUE FOLDS IN REVERSE SO REVERSE INCREMENTS
  blue_up_propel[11] += UP_TICKS_PROPEL_SMALL;      // Up LEG 4
  blue_up_propel[8] -= UP_TICKS_PROPEL_SMALL;      // Up LEG 3

  std::cout << "GENERATING FOR YELLOW LEGS\n";
  yellow_up_propel[5] -= UP_TICKS_PROPEL_SMALL;      // Up LEG 4
  yellow_up_propel[2] += UP_TICKS_PROPEL_SMALL;      // Up LEG 3

}

// WALKING
#define UP_MOTOR2  3074  // Up
// ROLLING
#define DOWN_MOTOR2  1020  // Down (for rolling)

// WALKING
// #define DOWN_MOTOR5  2042  // Down (flat no under)
#define UP_MOTOR5  1039  // Up
// ROLLING
#define DOWN_MOTOR5  2919  // Down (for rolling)

#define DOWN_MOTOR8  2051  // Down (flat no under)
#define UP_MOTOR8  3204  // Up

#define DOWN_MOTOR11 2044  // Down (flat no under)
#define UP_MOTOR11  957  // Up

// Yaw movement (1, 4, 7, 10) (CLOCKWISE & COUNTER-CLOCKWISE)
#define CLOCKWISE_MOTOR1       1858
#define COUNTER_CLOCKWISE_MOTOR1 3010

#define CLOCKWISE_MOTOR4       1063
#define COUNTER_CLOCKWISE_MOTOR4 2236

#define CLOCKWISE_MOTOR7       1877
#define COUNTER_CLOCKWISE_MOTOR7 3092

#define CLOCKWISE_MOTOR10      2004
#define COUNTER_CLOCKWISE_MOTOR10 3215

// Fold movement (3, 6, 9, 12) (CLOCKWISE & COUNTER-CLOCKWISE)
#define CLOCKWISE_MOTOR3       990
#define COUNTER_CLOCKWISE_MOTOR3 3072

#define CLOCKWISE_MOTOR6       1028
#define COUNTER_CLOCKWISE_MOTOR6 3080

#define CLOCKWISE_MOTOR9       3082
#define COUNTER_CLOCKWISE_MOTOR9 1024

#define CLOCKWISE_MOTOR12      3093 
#define COUNTER_CLOCKWISE_MOTOR12 1041

int degree_to_pos_diff(int degree) {
  return static_cast<int>((degree/360.0) * 4095);   // used 360.0 to prevent zero for small angles
}

int getch()
{
#if defined(__linux__) || defined(__APPLE__)
  struct termios oldt, newt;
  int ch;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
#elif defined(_WIN32) || defined(_WIN64)
  return _getch();
#endif
}

int kbhit(void)
{
#if defined(__linux__) || defined(__APPLE__)
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if (ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }

  return 0;
#elif defined(_WIN32) || defined(_WIN64)
  return _kbhit();
#endif
}


void update_present_positions(dynamixel::GroupSyncRead &groupSyncRead, 
                 dynamixel::PacketHandler *packetHandler, 
                 dynamixel::PortHandler *portHandler)
{
  // Clear previous parameters
  groupSyncRead.clearParam();

  // Scan for active motors
  for (int id = 1; id <= NUM_MOTORS; id++) {
      int dxl_comm_result = packetHandler->ping(portHandler, id);
      if (dxl_comm_result == COMM_SUCCESS) {
          // Add ID to GroupSyncRead
          bool dxl_addparam_result = groupSyncRead.addParam(id);
      }
  }

  // Read all present positions
  int dxl_comm_result = groupSyncRead.txRxPacket();

  // Update present positions of connected motors
  for (int id = 1; id <= NUM_MOTORS; id++) {
      if (groupSyncRead.isAvailable(id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)) {
          int32_t position = groupSyncRead.getData(id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
          present_positions[id] = position;
      }
  }

}

void move_to(
          int* positions,
          dynamixel::GroupSyncWrite &groupSyncWrite, 
          dynamixel::PacketHandler *packetHandler,
          dynamixel::GroupSyncRead &groupSyncRead,
          dynamixel::PortHandler *portHandler) 
{
  // Clear previous SyncWrite parameters
  groupSyncWrite.clearParam();

  for (int id = 1; id <= 12; id++)  // Loop through motor IDs 1-12
  {
      uint8_t param_goal_position[4];
      int goal_position = positions[id];

      param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(goal_position));
      param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(goal_position));
      param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(goal_position));
      param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(goal_position));

      // Add goal position to SyncWrite buffer
      if (!groupSyncWrite.addParam(id, param_goal_position)) {
          fprintf(stderr, "[ID:%03d] groupSyncWrite addParam failed\n", id);
          continue;
      }
  }
  // Transmit the home positions to all motors at once
  int dxl_comm_result = groupSyncWrite.txPacket();
  if (dxl_comm_result != COMM_SUCCESS) {
      printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  }

  // Clear SyncWrite buffer after sending data
  groupSyncWrite.clearParam();

  std::this_thread::sleep_for(std::chrono::milliseconds(10));  // Allow TIME for motors to reach the position
  update_present_positions(groupSyncRead, packetHandler, portHandler);

  // Print the updated positions for debugging
  // std::cout << "Updated Present Positions:\n";
  // for (int id = 1; id <= NUM_MOTORS; id++) {
  //     std::cout << "[ID:" << id << "] Position: " << present_positions[id] << "\n";
  // }
  // std::cout << "\n";
}

void move_to_target_positions(
                      int* target_positions,
                      dynamixel::GroupSyncWrite &groupSyncWrite, 
                      dynamixel::PacketHandler *packetHandler 
                      )
{
  // Clear previous SyncWrite parameters
  groupSyncWrite.clearParam();

  for (int id = 1; id <= 12; id++)  // Loop through motor IDs 1-12
  {
      uint8_t param_goal_position[4];
      int goal_position = target_positions[id];

      param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(goal_position));
      param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(goal_position));
      param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(goal_position));
      param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(goal_position));

      // Add goal position to SyncWrite buffer
      if (!groupSyncWrite.addParam(id, param_goal_position)) {
          fprintf(stderr, "[ID:%03d] groupSyncWrite addParam failed\n", id);
          continue;
      }
  }

  // Transmit the target positions to all motors at once
  int dxl_comm_result = groupSyncWrite.txPacket();
  if (dxl_comm_result != COMM_SUCCESS) {
      printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  }

  // Clear SyncWrite buffer after sending data
  groupSyncWrite.clearParam();
}

std::string get_random_command() {
    static std::random_device rd;   // Non-deterministic random seed
    static std::mt19937 gen(rd());  // Mersenne Twister RNG
    static std::uniform_int_distribution<int> dist(0, 1); // Randomly pick 0 or 1

    return dist(gen) ? "rpy" : "rpb"; // Return "rpy" if 1, "rpb" if 0
}

int main() 
{
  generate_movement_arrays_roll_fw();

  // Initialize PortHandler instance
  // Set the port path
  // Get methods and members of PortHandlerLinux or PortHandlerWindows
  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

  // Initialize PacketHandler instance
  // Set the protocol version
  // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Initialize GroupSyncWrite instance
  dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION);

  // Initialize Groupsyncread instance for Present Position
  dynamixel::GroupSyncRead groupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);

  int dxl_comm_result = COMM_TX_FAIL;               // Communication result
  bool dxl_addparam_result = false;                 // addParam result

  uint8_t dxl_error = 0;                            // Dynamixel error

  // Open port
  if (portHandler->openPort())
  {
    printf("Succeeded to open the port!\n");
  }
  else
  {
    printf("Failed to open the port!\n");
    printf("Press any key to terminate...\n");
    getch();
    return 0;
  }

  // Set port baudrate
  if (portHandler->setBaudRate(BAUDRATE))
  {
    printf("Succeeded to change the baudrate!\n");
  }
  else
  {
    printf("Failed to change the baudrate!\n");
    printf("Press any key to terminate...\n");
    getch();
    return 0;
  }


  for (int i = 0; i < 20; i++) {
    DXL_ID = i;
    // Enable Dynamixel Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
      printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }
    else
    {
      printf("Dynamixel#%d has been successfully connected \n", DXL_ID);
    }

    // Add parameter storage for Dynamixel#1 present position value
    dxl_addparam_result = groupSyncRead.addParam(DXL_ID);
    if (dxl_addparam_result != true)
    {
      fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", DXL_ID);
      return 0;
    }
  }

    int file;
    int adapter_nr = 1; // use /dev/i2c-1
    char filename[20];
    snprintf(filename, 19, "/dev/i2c-%d", adapter_nr);
    file = open(filename, O_RDWR);
    if (file < 0) {
        std::cerr << "Failed to open I2C bus\n";
        return 1;
    }

    int addr = 0x6A; // LSM330DHCX I2C address
    if (ioctl(file, I2C_SLAVE, addr) < 0) {
        std::cerr << "Failed to set I2C address\n";
        close(file);
        return 1;
    }

    // Enable gyroscope and accelerometer
    write_register(file, 0x10, 0x60); // Accelerometer
    write_register(file, 0x11, 0x60); // Gyroscope
    sleep(1); // Wait for sensor to initialize

    // Bias offsets
    float accel_z_offset = 0.2;
    float gyro_z_offset = -0.37;

    //  // Open CSV file for writing
    // std::ofstream csvFile("imu_data_no_gaining_momentum.csv");
    // if (!csvFile.is_open()) {
    //     std::cerr << "Failed to open CSV file\n";
    //     close(file);
    //     return 1;
    // }

    // // Write CSV headers
    // csvFile << "Timestamp,Gyro_X_dps,Gyro_Y_dps,Gyro_Z_dps,Accel_X_mps2,Accel_Y_mps2,Accel_Z_mps2\n";


    auto start_time = std::chrono::high_resolution_clock::now();

    // Thresholds based on data analysis
    const float accel_z_threshold = 9.5;    // m/s² for a successful propel
    const float gyro_y_stability = 6.5;     // rad/s threshold for orientation stability
    const int window_size = 2;            // Number of samples to average (for smoothing)
    
    // Variables for data accumulation
    float accumulated_tilt_angle = 0;

    int sample_count = 0;
    
  while (true) {
    std::string command = "rpy";

    // Read gyroscope data
    int16_t gyro_x = read_16bit_register(file, 0x22, 0x23);
    int16_t gyro_y = read_16bit_register(file, 0x24, 0x25);
    int16_t gyro_z = read_16bit_register(file, 0x26, 0x27);

    // Read accelerometer data
    int16_t accel_x = read_16bit_register(file, 0x28, 0x29);
    int16_t accel_y = read_16bit_register(file, 0x2A, 0x2B);
    int16_t accel_z = read_16bit_register(file, 0x2C, 0x2D);


    // Apply scale factors
    float gyro_dps_x = gyro_x * (250.0 / 32768.0);
    float gyro_dps_y = gyro_y * (250.0 / 32768.0);
    float gyro_dps_z = (gyro_z * (250.0 / 32768.0)) - gyro_z_offset;

    float accel_mps2_x = accel_x * (2.0 / 32768.0) * 9.81;
    float accel_mps2_y = accel_y * (2.0 / 32768.0) * 9.81;
    float accel_mps2_z = ((accel_z * (2.0 / 32768.0)) * 9.81) - accel_z_offset;

    // Compute tilt angle around x-axis
    float angle_rad = std::atan2(accel_mps2_y, accel_mps2_z);
    float angle_degrees = angle_rad * (180.0 / M_PI);

    // Timestamp for each reading
    auto current_time = std::chrono::high_resolution_clock::now();
    double timestamp = std::chrono::duration<double>(current_time - start_time).count();

    // std::cout << "Gyro Raw - X: " << gyro_x << " Y: " << gyro_y << " Z: " << gyro_z << " | "
    // << "Accel Raw - X: " << accel_x << " Y: " << accel_y << " Z: " << accel_z << std::endl;
    
    // Write to CSV
    // csvFile << std::fixed << std::setprecision(6)
    //         << timestamp << ","
    //         << gyro_dps_x << "," << gyro_dps_y << "," << gyro_dps_z << ","
    //         << accel_mps2_x << "," << accel_mps2_y << "," << accel_mps2_z << "\n";

    // csvFile.flush(); // Ensure data is written in real-time


    // Accumulate data for smoothing
    accumulated_tilt_angle += angle_degrees;

    sample_count++;
    // std::cout << "Sample Count: " << sample_count << std::endl;

    // Check if enough samples have been collected
    if (sample_count >= window_size) {        
        float avg_tilt_angle = accumulated_tilt_angle / sample_count;
        std::cout << "Average Tilt Angle: " << avg_tilt_angle << " degrees" << std::endl;
         // Use tilt angle to determine which side is under
        bool blue_under = (avg_tilt_angle >= -180 && avg_tilt_angle <= -122) ||
        (avg_tilt_angle >= 123 && avg_tilt_angle <= 180);     // Positive rotation → blue under
        bool yellow_under = avg_tilt_angle >= -54 && avg_tilt_angle <= 58;  // Negative rotation → yellow under

        // Check propulsion conditions
        if (yellow_under) {
          std::cout << "Yellow under – pushing yellow\n";
          move_to(yellow_up_propel, groupSyncWrite, packetHandler, groupSyncRead, portHandler);
          std::this_thread::sleep_for(std::chrono::milliseconds(700));
          move_to(perfect_cir, groupSyncWrite, packetHandler, groupSyncRead, portHandler);
        } else if (blue_under) {
          std::cout << "Blue under – pushing blue\n";
          move_to(blue_up_propel, groupSyncWrite, packetHandler, groupSyncRead, portHandler);
          std::this_thread::sleep_for(std::chrono::milliseconds(700));
          move_to(perfect_cir, groupSyncWrite, packetHandler, groupSyncRead, portHandler);
        } else {
          std::cout << "Unknown orientation. Executing random roll...\n";
          std::string random_cmd = get_random_command();
          if (random_cmd == "rpy") {
              move_to(yellow_up_propel, groupSyncWrite, packetHandler, groupSyncRead, portHandler);
          } else {
              move_to(blue_up_propel, groupSyncWrite, packetHandler, groupSyncRead, portHandler);
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(700));
          move_to(perfect_cir, groupSyncWrite, packetHandler, groupSyncRead, portHandler);
      }

        // Reset accumulators
        // after the window size is met, instead of prematurely which is immediately after the decision
        accumulated_tilt_angle = 0;
        sample_count = 0;
    }

    // ROLL PROPEL FW
    if (command == "rpy") {
      move_to(perfect_cir, groupSyncWrite, packetHandler,groupSyncRead, portHandler); 
      const int NUM_MOVEMENTS = 2;

      int* roll_fw_movements[NUM_MOVEMENTS] = {
        yellow_up_propel,
        perfect_cir
      };
      
      for (int i = 0; i < NUM_MOVEMENTS; i++) {
        move_to(roll_fw_movements[i], groupSyncWrite, packetHandler, groupSyncRead, portHandler);
        std::this_thread::sleep_for(std::chrono::milliseconds(700));  
      }
    }
    else if (command == "rpb") {
      move_to(perfect_cir, groupSyncWrite, packetHandler,groupSyncRead, portHandler); 

      const int NUM_MOVEMENTS = 2;
      int* roll_fw_movements[NUM_MOVEMENTS] = {
        blue_up_propel,
        perfect_cir
      };
      
      for (int i = 0; i < NUM_MOVEMENTS; i++) {
        move_to(roll_fw_movements[i], groupSyncWrite, packetHandler, groupSyncRead, portHandler);
        std::this_thread::sleep_for(std::chrono::milliseconds(700));  
      }
    }

    // Adjust sampling rate (lower value for higher frequency)
    // usleep(2000); // 10ms delay (~100Hz sampling rate)

  }

// csvFile.close();
// Close port
portHandler->closePort();

return 0;

}
