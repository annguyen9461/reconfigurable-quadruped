// === Standard C/C++ Libraries ===
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <iostream>
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
int present_positions[NUM_MOTORS + 1] = {0};

// === ROLLING CONFIGURATIONS ===
const double TICKS_PER_DEGREE = 4096.0 / 360.0;  // ≈ 11.37778
const int UP_DOWN_TICKS_ROLL = static_cast<int>(50 * TICKS_PER_DEGREE); 
const int UP_TICKS_PROPEL_SMALL = static_cast<int>(30 * TICKS_PER_DEGREE);

// Circle Positions - Perfect rolling circle configuration
int perfect_cir[NUM_MOTORS + 1] = {0, 
  2040,  // [ID:1]
  1098,  // [ID:2]
  3081,  // [ID:3]
  2054,  // [ID:4]
  2997,  // [ID:5]
  1007,  // [ID:6]
  2041,  // [ID:7]
  2993,  // [ID:8]
  1045,  // [ID:9]
  3054,  // [ID:10]
  1095,  // [ID:11]
  3091   // [ID:12]
};

// BLUE - Leg 3 and 4 Movements
int blue_up_cir[NUM_MOTORS + 1] = {0}; 
int blue_up_propel[NUM_MOTORS + 1] = {0}; 

// YELLOW - Leg 1 and 2 Movements
int yellow_up_cir[NUM_MOTORS + 1] = {0}; 
int yellow_up_propel[NUM_MOTORS + 1] = {0}; 

// Function to copy array
void copy_array(int* dest, int* src) {
    for (int i = 0; i <= NUM_MOTORS; i++) {
        dest[i] = src[i];
    }
}

// Function to populate movement arrays for rolling
void generate_movement_arrays_roll_fw() {
  // Start with perfect_cir for all movements
  copy_array(blue_up_cir, perfect_cir);
  copy_array(blue_up_propel, perfect_cir);
  copy_array(yellow_up_cir, perfect_cir);
  copy_array(yellow_up_propel, perfect_cir);
  
  // BLUE LEGS (3 & 4)
  blue_up_cir[11] += UP_DOWN_TICKS_ROLL;      // Up LEG 4
  blue_up_cir[8] -= UP_DOWN_TICKS_ROLL;       // Up LEG 3

  blue_up_propel[11] += UP_TICKS_PROPEL_SMALL;  // Up LEG 4 (smaller movement for propelling)
  blue_up_propel[8] -= UP_TICKS_PROPEL_SMALL;   // Up LEG 3 (smaller movement for propelling)

  // YELLOW LEGS (1 & 2)
  yellow_up_cir[5] -= UP_DOWN_TICKS_ROLL;      // Up LEG 2
  yellow_up_cir[2] += UP_DOWN_TICKS_ROLL;      // Up LEG 1

  yellow_up_propel[5] -= UP_TICKS_PROPEL_SMALL;  // Up LEG 2 (smaller movement for propelling)
  yellow_up_propel[2] += UP_TICKS_PROPEL_SMALL;  // Up LEG 1 (smaller movement for propelling)
}

// Minimal keyboard input functions
int getch() {
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

// Update positions from hardware
void update_present_positions(dynamixel::GroupSyncRead &groupSyncRead, 
                 dynamixel::PacketHandler *packetHandler, 
                 dynamixel::PortHandler *portHandler) {
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

// Send movement commands to motors
void move_to(
          int* positions,
          dynamixel::GroupSyncWrite &groupSyncWrite, 
          dynamixel::PacketHandler *packetHandler,
          dynamixel::GroupSyncRead &groupSyncRead,
          dynamixel::PortHandler *portHandler) {
  // Clear previous SyncWrite parameters
  groupSyncWrite.clearParam();

  for (int id = 1; id <= 12; id++) {
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
  // Transmit the positions to all motors at once
  int dxl_comm_result = groupSyncWrite.txPacket();
  if (dxl_comm_result != COMM_SUCCESS) {
      printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  }

  // Clear SyncWrite buffer after sending data
  groupSyncWrite.clearParam();

  std::this_thread::sleep_for(std::chrono::milliseconds(10));  // Allow time for motors to reach the position
  update_present_positions(groupSyncRead, packetHandler, portHandler);
}

// Helper function to get a random command
std::string get_random_command() {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_int_distribution<int> dist(0, 1);

    return dist(gen) ? "rfy" : "rfb"; // Return "rfy" if 1, "rfb" if 0
}

int main() {
  // Generate movement arrays for rolling
  generate_movement_arrays_roll_fw();

  // Initialize PortHandler instance
  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

  // Initialize PacketHandler instance
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Initialize GroupSyncWrite instance
  dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION);

  // Initialize Groupsyncread instance for Present Position
  dynamixel::GroupSyncRead groupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);

  int dxl_comm_result = COMM_TX_FAIL;
  uint8_t dxl_error = 0;

  // Open port
  if (portHandler->openPort()) {
    printf("Succeeded to open the port!\n");
  } else {
    printf("Failed to open the port!\n");
    printf("Press any key to terminate...\n");
    getch();
    return 0;
  }

  // Set port baudrate
  if (portHandler->setBaudRate(BAUDRATE)) {
    printf("Succeeded to change the baudrate!\n");
  } else {
    printf("Failed to change the baudrate!\n");
    printf("Press any key to terminate...\n");
    getch();
    return 0;
  }

  // Enable Dynamixel Torque for all motors
  for (int i = 1; i <= NUM_MOTORS; i++) {
    DXL_ID = i;
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    if (dxl_comm_result == COMM_SUCCESS && dxl_error == 0) {
      printf("Dynamixel #%d has been successfully connected\n", DXL_ID);
    }

    // Add parameter storage for present position value
    bool dxl_addparam_result = groupSyncRead.addParam(DXL_ID);
    if (dxl_addparam_result != true) {
      fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", DXL_ID);
      return 0;
    }
  }

  // Set up I2C for IMU
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

  // Bias offsets for sensor calibration
  float accel_z_offset = 0.2;
  float gyro_z_offset = -0.37;

  auto start_time = std::chrono::high_resolution_clock::now();

  // Thresholds and parameters for orientation detection
  const float accel_z_threshold = 9.5;    // m/s² for a successful propel
  const float gyro_y_stability = 6.5;     // rad/s threshold for orientation stability
  const int window_size = 2;              // Number of samples to average (for smoothing)
  
  // Variables for data accumulation
  float accumulated_tilt_angle = 0;
  int sample_count = 0;
  
  // Move to perfect circle configuration initially
  move_to(perfect_cir, groupSyncWrite, packetHandler, groupSyncRead, portHandler);
  
  while (true) {
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

    // Accumulate data for smoothing
    accumulated_tilt_angle += angle_degrees;
    sample_count++;

    // Check if enough samples have been collected
    if (sample_count >= window_size) {
      float avg_tilt_angle = accumulated_tilt_angle / sample_count;
      std::cout << "Average Tilt Angle: " << avg_tilt_angle << " degrees" << std::endl;
      
      // Use tilt angle to determine which side is under
      bool blue_under = (avg_tilt_angle >= -180 && avg_tilt_angle <= -122) ||
                        (avg_tilt_angle >= 123 && avg_tilt_angle <= 180);
      bool yellow_under = avg_tilt_angle >= -54 && avg_tilt_angle <= 58;

      // Execute appropriate rolling action based on orientation
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
        if (random_cmd == "rfy") {
          move_to(yellow_up_propel, groupSyncWrite, packetHandler, groupSyncRead, portHandler);
        } else {
          move_to(blue_up_propel, groupSyncWrite, packetHandler, groupSyncRead, portHandler);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(700));
        move_to(perfect_cir, groupSyncWrite, packetHandler, groupSyncRead, portHandler);
      }

      // Reset accumulators
      accumulated_tilt_angle = 0;
      sample_count = 0;
    }
  }

  // Close port
  portHandler->closePort();
  close(file);
  return 0;
}