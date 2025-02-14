/*******************************************************************************
* Copyright 2017 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Author: Ryu Woon Jung (Leon) */

//
// *********     Sync Read and Sync Write Example      *********
//
//
// Available Dynamixel model on this example : All models using Protocol 2.0
// This example is tested with two Dynamixel PRO 54-200, and an USB2DYNAMIXEL
// Be sure that Dynamixel PRO properties are already set as %% ID : 1 / Baudnum : 1 (Baudrate : 57600)
//

#if defined(__linux__) || defined(__APPLE__)
#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

#include <stdlib.h>
#include <stdio.h>

#include "dynamixel_sdk.h"                                  // Uses Dynamixel SDK library

#include <string.h>
#include <chrono>
#include <thread>

#include <unordered_map>
#include <vector>
#include <sstream>
#include <iostream>

#define MAX_INPUT_SIZE 100      // define max input buffer size

// Control table address
#define ADDR_PRO_TORQUE_ENABLE          64                 // Control table address is different in Dynamixel model
#define ADDR_PRO_GOAL_POSITION          116
#define ADDR_PRESENT_POSITION           132

// Data Byte Length
#define LEN_PRO_GOAL_POSITION           4
#define LEN_PRESENT_POSITION            4

// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define BAUDRATE                        57600
#define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MINIMUM_POSITION_VALUE      0             // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE      4095              // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD     20                  // Dynamixel moving status threshold

#define ESC_ASCII_VALUE                 0x1b

#define NUM_MOTORS                      12                  // IDs from 1 to 12


int DXL_ID;
bool toggle_position = false;  // Toggles between the two positions
bool forward_running = false;

void scan_motors(dynamixel::GroupSyncRead &groupSyncRead, 
                 dynamixel::PacketHandler *packetHandler, 
                 dynamixel::PortHandler *portHandler);

void set_torque(dynamixel::PacketHandler *packetHandler, 
                dynamixel::PortHandler *portHandler, 
                const char *command, char *ids_str); 

int present_positions[NUM_MOTORS + 1] = {0, 
  2045, 2053, 2057, 2054, 2035, 1014, 2044, 2047, 1027, 3051, 2043, 1056
};

int aligned_before_rolling[NUM_MOTORS + 1] = {0, 
  2045, 2053, 2057, 2054, 2035, 1014, 2044, 2047, 1027, 3051, 2043, 1056
};

// Up and Down movement (2, 5, 8, 11) (ROLL)
#define DOWN_MOTOR2  2048  // Down (flat no under)
#define UP_MOTOR2  3074  // Up

#define DOWN_MOTOR5  2042  // Down (flat no under)
#define UP_MOTOR5  1039  // Up

#define DOWN_MOTOR8  2051  // Down (flat no under)
#define UP_MOTOR8  3204  // Up

#define DOWN_MOTOR11 2044  // Down (flat no under)
#define UP_MOTOR11  957  // Up

// Side to side movement (1,4,7,10) (YAW)
#define PERPEN_MOTOR1  2046  // Perpendicular to body
#define PARA_MOTOR1  3058  // Parallel body along LONG side

#define PERPEN_MOTOR4  2054  // Perpendicular to body
#define PARA_MOTOR4  1058  // Parallel body along LONG side

#define PERPEN_MOTOR7  2044  // Perpendicular to body
#define PARA_MOTOR7  3084  // Parallel body along LONG side

#define PERPEN_MOTOR10 3051  // Perpendicular to body
#define PARA_MOTOR10 2042  // Parallel body along LONG side

// Yaw movement (1, 4, 7, 10) (CLOCKWISE & COUNTER-CLOCKWISE)
#define CLOCKWISE_MOTOR1       1858
#define COUNTER_CLOCKWISE_MOTOR1 3010

#define CLOCKWISE_MOTOR4       1063
#define COUNTER_CLOCKWISE_MOTOR4 2236

#define CLOCKWISE_MOTOR7       1877
#define COUNTER_CLOCKWISE_MOTOR7 3092

#define CLOCKWISE_MOTOR10      2004
#define COUNTER_CLOCKWISE_MOTOR10 3215


// Up and Down movement (2, 5, 8, 11) (ROLL)
#define MIN_ANGLE_MOTOR2  (2048.0 / 4095.0 * 360.0)  // ≈ 180.0 degrees
#define MAX_ANGLE_MOTOR2  (3074.0 / 4095.0 * 360.0)  // ≈ 270.6 degrees

#define MIN_ANGLE_MOTOR5  (2042.0 / 4095.0 * 360.0)  // ≈ 179.5 degrees
#define MAX_ANGLE_MOTOR5  (1039.0 / 4095.0 * 360.0)  // ≈ 91.3 degrees

#define MIN_ANGLE_MOTOR8  (2051.0 / 4095.0 * 360.0)  // ≈ 180.3 degrees
#define MAX_ANGLE_MOTOR8  (3204.0 / 4095.0 * 360.0)  // ≈ 281.5 degrees

#define MIN_ANGLE_MOTOR11 (2044.0 / 4095.0 * 360.0)  // ≈ 179.6 degrees
#define MAX_ANGLE_MOTOR11 (957.0 / 4095.0 * 360.0)   // ≈ 84.0 degrees

// Side to side movement (1,4,7,10) (YAW)
#define MIN_ANGLE_MOTOR1  (2046.0 / 4095.0 * 360.0)  // ≈ 180.0 degrees
#define MAX_ANGLE_MOTOR1  (3058.0 / 4095.0 * 360.0)  // ≈ 269.0 degrees

#define MIN_ANGLE_MOTOR4  (2054.0 / 4095.0 * 360.0)  // ≈ 180.5 degrees
#define MAX_ANGLE_MOTOR4  (1058.0 / 4095.0 * 360.0)  // ≈ 93.0 degrees

#define MIN_ANGLE_MOTOR7  (2044.0 / 4095.0 * 360.0)  // ≈ 179.6 degrees
#define MAX_ANGLE_MOTOR7  (3084.0 / 4095.0 * 360.0)  // ≈ 271.1 degrees

#define MIN_ANGLE_MOTOR10 (3051.0 / 4095.0 * 360.0)  // ≈ 268.5 degrees
#define MAX_ANGLE_MOTOR10 (2042.0 / 4095.0 * 360.0)  // ≈ 179.5 degrees


// Struct to store both motors per leg (roll and yaw)
struct LegMotors {
  int roll_motor_id;     // Motor responsible for up/down
  int yaw_motor_id;      // Motor responsible for side-to-side
  int roll_down;         // Min position for roll movement
  int roll_up;           // Max position for roll movement
  int yaw_cw;            // Min position for yaw (CLOCKWISE)
  int yaw_ccw;           // Max position for yaw (COUNTER-CLOCKWISE)
};

// Map each leg number to its corresponding motors (using defines)
std::unordered_map<int, LegMotors> leg_motor_map = {
  {1, {2, 1, DOWN_MOTOR2, UP_MOTOR2, CLOCKWISE_MOTOR1, COUNTER_CLOCKWISE_MOTOR1}},  
  {2, {5, 4, DOWN_MOTOR5, UP_MOTOR5, CLOCKWISE_MOTOR4, COUNTER_CLOCKWISE_MOTOR4}},  
  {3, {8, 7, DOWN_MOTOR8, UP_MOTOR8, CLOCKWISE_MOTOR7, COUNTER_CLOCKWISE_MOTOR7}},  
  {4, {11, 10, DOWN_MOTOR11, UP_MOTOR11, CLOCKWISE_MOTOR10, COUNTER_CLOCKWISE_MOTOR10}}
};

int degree_to_pos_diff(int degree) {
  return static_cast<int>((degree/360.0) * 4095);   // used 360.0 to prevent zero for small angles
}

// Moves the motor CLOCKWISE by a given degree amount (YAW motor)
int go_clockwise(int leg_num, int degree) {
    LegMotors motors = leg_motor_map[leg_num];
  int yaw_cw = motors.yaw_cw;     // Clockwise position
  int yaw_ccw = motors.yaw_ccw;   // Counter-clockwise position
  int diff = degree_to_pos_diff(degree);
  int curr_pos_motor = present_positions[motors.yaw_motor_id];

  // Print debug information
  std::cout << "Leg " << leg_num << ":\n";
  std::cout << "  Yaw Motor ID: " << motors.yaw_motor_id << "\n";
  std::cout << "  Roll Motor ID: " << motors.roll_motor_id << "\n";
  std::cout << "  Current Yaw Position: " << curr_pos_motor << "\n";
  std::cout << "  Clockwise Limit: " << yaw_cw << "\n";
  std::cout << "  Counter-Clockwise Limit: " << yaw_ccw << "\n";
  std::cout << "  Degree to Move: " << degree << "\n";
  std::cout << "  Position Difference: " << diff << "\n";

  // Ensure position stays within limits when moving CLOCKWISE
  if (yaw_ccw > yaw_cw) {
      return std::max(curr_pos_motor + diff, yaw_cw);
  } else {
      return std::min(curr_pos_motor - diff, yaw_cw);
  }
}

// Moves the motor COUNTER-CLOCKWISE by a given degree amount (YAW motor)
int go_counter_clockwise(int leg_num, int degree) {
  LegMotors motors = leg_motor_map[leg_num];
  int yaw_cw = motors.yaw_cw;
  int yaw_ccw = motors.yaw_ccw;
  int diff = degree_to_pos_diff(degree);
  int curr_pos_motor = present_positions[motors.yaw_motor_id];

  // Ensure position stays within limits when moving COUNTER-CLOCKWISE
  if (yaw_ccw > yaw_cw) {
      return std::min(curr_pos_motor - diff, yaw_ccw);
  } else {
      return std::max(curr_pos_motor + diff, yaw_ccw);
  }
}

// Moves the motor UP by a given degree amount (ROLL motor)
int go_up(int leg_num, int degree) {
  LegMotors motors = leg_motor_map[leg_num];
  int roll_up = motors.roll_up;
  int roll_down = motors.roll_down;
  int diff = degree_to_pos_diff(degree);
  int curr_pos_motor = present_positions[motors.roll_motor_id];

  // Ensure position stays within limits when moving UP
  if (roll_up > roll_down) {
      return std::min(curr_pos_motor + diff, roll_up);
  } else {
      return std::max(curr_pos_motor - diff, roll_up);
  }
}

// Moves the motor DOWN by a given degree amount (ROLL motor)
int go_down(int leg_num, int degree) {
  LegMotors motors = leg_motor_map[leg_num];
  int roll_up = motors.roll_up;
  int roll_down = motors.roll_down;
  int diff = degree_to_pos_diff(degree);
  int curr_pos_motor = present_positions[motors.roll_motor_id];

  // Ensure position stays within limits when moving UP
  if (roll_up > roll_down) {
      return std::max(curr_pos_motor - diff, roll_down);
  } else {
      return std::min(curr_pos_motor + diff, roll_down);
  }
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

void scan_motors(dynamixel::GroupSyncRead &groupSyncRead, 
                 dynamixel::PacketHandler *packetHandler, 
                 dynamixel::PortHandler *portHandler)
{
  printf("Scanning for connected Dynamixel motors...\n");

  // Clear previous parameters
  groupSyncRead.clearParam();
  
  // Scan for active motors
  for (int id = 1; id <= NUM_MOTORS; id++) {
      int dxl_comm_result = packetHandler->ping(portHandler, id);
      if (dxl_comm_result == COMM_SUCCESS) {
          printf("Found Dynamixel ID: %d\n", id);

          // Add ID to GroupSyncRead
          bool dxl_addparam_result = groupSyncRead.addParam(id);
          if (!dxl_addparam_result) {
              printf("Failed to add ID %d to SyncRead\n", id);
          }
      }
  }

  // Read all present positions
  int dxl_comm_result = groupSyncRead.txRxPacket();
  if (dxl_comm_result != COMM_SUCCESS) {
      printf("Failed to read positions: %s\n", packetHandler->getTxRxResult(dxl_comm_result));
  }

  // Print present positions of connected motors
  printf("\nCurrent Positions:\n");
  for (int id = 1; id <= NUM_MOTORS; id++) {
      if (groupSyncRead.isAvailable(id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)) {
          int32_t position = groupSyncRead.getData(id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
          printf("[ID:%d] Position: %d\n", id, position);
      }
  }
  printf("\n");
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
  // printf("Present motor positions UPDATED\n");
  // for (int id = 1; id <= NUM_MOTORS; id++) {
  //   printf("[ID: %d] Position: %d\n", id, present_positions[id]);
  // }
}

void set_torque(dynamixel::PacketHandler *packetHandler, 
                dynamixel::PortHandler *portHandler, 
                const char *command, char *ids_str)
{
  bool enable = (strcmp(command, "en") == 0) ? TORQUE_ENABLE : TORQUE_DISABLE;
  uint8_t dxl_error = 0;

  char *token = strtok(ids_str, " ");
  while (token != NULL) {
      int dxl_id = atoi(token);
      if (dxl_id < 1 || dxl_id > NUM_MOTORS) {
          printf("Invalid ID: %d. Must be between 1 and %d.\n", dxl_id, NUM_MOTORS);
      } else {
          int dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id, ADDR_PRO_TORQUE_ENABLE, enable, &dxl_error);
          if (dxl_comm_result != COMM_SUCCESS) {
              printf("[ID:%d] Torque change failed: %s\n", dxl_id, packetHandler->getTxRxResult(dxl_comm_result));
          } else {
              printf("[ID:%d] Torque %s\n", dxl_id, enable ? "ENABLED" : "DISABLED");
          }
      }
      token = strtok(NULL, " ");
  }
}

int home2_positions[NUM_MOTORS + 1] = {0, 
    1632, 2255, 2060, 2449, 1860, 1023, 661, 2219, 1014, 2514, 1862, 1064
};

int home2_perpen[NUM_MOTORS + 1] = {0, 
    1632, 2031, 2058, 2449, 2076, 1022, 661, 2008, 1015, 2513, 2062, 1060
};

int home_positions[NUM_MOTORS + 1] = {0, 
  1313, 2030, 2060, 2818, 2076, 1021, 305, 2007, 1016, 2868, 2066, 1062
};
int circle_positions[NUM_MOTORS + 1] = {0, 
  1062, 2947, 24, 3059, 1174, 3046, 22, 1096, 1004, 3089, 2943, 1055
};
int blue_folded_under_cir1[NUM_MOTORS + 1] = {0, 
    1059, 2945, 1201, 3060, 1175, 1897, 23, 1047, 1007, 3089, 2994, 1052
};
int yellow_folded_above_cir2[NUM_MOTORS + 1] = {0, 
    1062, 2957, 19, 3059, 1164, 3068, 23, 1057, 1010, 3088, 2986, 1051
};

// Roll Forward - Open Blue (Motors 8 and 11)
int roll_fw_open_blue[NUM_MOTORS + 1] = {0, 
    1062, 2996, 20, 3059, 1130, 3069, 23, 1685, 1010, 3088, 2381, 1050
};

// Roll Forward - Close Blue (Motors 8 and 11)
int roll_fw_close_blue[NUM_MOTORS + 1] = {0, 
    1062, 2996, 19, 3059, 1131, 3069, 23, 1026, 1010, 3088, 3084, 1047
};

// Roll Forward - Open Yellow (Motors 5 and 2)
int roll_fw_open_yellow[NUM_MOTORS + 1] = {0, 
    1062, 2505, 19, 3059, 1613, 3068, 25, 1085, 1015, 3089, 2986, 1056
};

// Roll Forward - Close Yellow (Motors 5 and 2)
int roll_fw_close_yellow[NUM_MOTORS + 1] = {0, 
    1062, 2969, 18, 3059, 1093, 3067, 23, 1101, 1010, 3088, 2978, 1051
};

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

  printf("All motors moved to goal position.\n");
  std::this_thread::sleep_for(std::chrono::milliseconds(100));  // Allow TIME for motors to reach the position
  update_present_positions(groupSyncRead, packetHandler, portHandler);

  // ✅ Print the updated positions for debugging
  std::cout << "Updated Present Positions:\n";
  for (int id = 1; id <= NUM_MOTORS; id++) {
      std::cout << "[ID:" << id << "] Position: " << present_positions[id] << "\n";
  }
  std::cout << "\n";
}

void move_to_target_positions(
                      int* target_positions,
                      dynamixel::GroupSyncWrite &groupSyncWrite, 
                      dynamixel::PacketHandler *packetHandler 
                      )
{
  printf("Moving motors to %s...\n", toggle_position ? "TOP RIGHT up" : "TOP LEFT up");

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

  printf("Motors moved to %s position.\n", toggle_position ? "TOP RIGHT up" : "TOP LEFT up");
}

void gradual_transition(int* cur_positions, 
                         int* next_positions, 
                         dynamixel::GroupSyncWrite &groupSyncWrite, 
                         dynamixel::PacketHandler *packetHandler,
                         dynamixel::GroupSyncRead &groupSyncRead,
                          dynamixel::PortHandler *portHandler) {
    const int step_size = 12;  // number of steps for smooth transition
    int step_arr[NUM_MOTORS + 1] = {0};

    int num_motors = NUM_MOTORS;

    // loop range to start from 1 (ignoring index 0)
    for (int i = 1; i <= num_motors; i++) {
        step_arr[i] = (next_positions[i] - cur_positions[i]) / step_size;
    }

    // perform transitions for each step
    for (int step = 0; step < step_size; step++) {
        // update motor transition based on respective step size
        for (int i = 1; i <= num_motors; i++) {
            cur_positions[i] += step_arr[i];
        }
        move_to(cur_positions, groupSyncWrite, packetHandler, groupSyncRead, portHandler);
    }

    // bool moving = true;
    // while (moving) {
    //   moving = false;

    //   for (int i = 1; i <= NUM_MOTORS; i++) {
    //     if (abs(next_positions[i] - cur_positions[i]) > abs(step_arr[i])) {
    //         cur_positions[i] += step_arr[i];
    //         moving = true;  // Keep moving if any motor hasn't reached its goal
    //     } else {
    //         cur_positions[i] = next_positions[i];  // Final correction
    //     }
    //   }

    //   move_to(cur_positions, groupSyncWrite, packetHandler, groupSyncRead, portHandler);
    //   std::this_thread::sleep_for(std::chrono::milliseconds(50));  // Add delay for smooth transition
    // }

    // ensure final position is accurate (due to integer division)
    move_to(next_positions, groupSyncWrite, packetHandler, groupSyncRead, portHandler);
}

void move_forward(dynamixel::GroupSyncWrite &groupSyncWrite, 
                  dynamixel::PacketHandler *packetHandler) 
{
  // Define the two alternating positions
  int top_left_lift[NUM_MOTORS + 1] = {0, 
      1779, 2283, 2048, 2398, 2105, 1056, 224, 2283, 1000, 2959, 2046, 1013
  };
  int top_left_fw[NUM_MOTORS + 1] = {0, 
      1779, 2010, 2048, 2398, 2105, 1056, 224, 2035, 1000, 2959, 2046, 1013
  };

  int top_right_lift[NUM_MOTORS + 1] = {0, 
      1060, 2015, 2045, 2940, 1788, 1057, 711, 2032, 1000, 2387, 1814, 1009
  };
  int top_right_fw[NUM_MOTORS + 1] = {0, 
      1060, 2015, 2045, 2940, 2107, 1057, 711, 2032, 1000, 2387, 2101, 1009
  };

  int* positions[] = {top_left_lift, top_left_fw, top_right_lift, top_right_fw};

  // Toggle between the two position sets
  if (toggle_position) {
    move_to_target_positions(top_left_lift, groupSyncWrite, packetHandler);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));  // Delay for stability
    move_to_target_positions(top_left_fw, groupSyncWrite, packetHandler);
  } else {
    move_to_target_positions(top_right_lift, groupSyncWrite, packetHandler);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));  // Delay for stability
    move_to_target_positions(top_right_fw, groupSyncWrite, packetHandler);
  }
  
  // Toggle between the two position sets
  toggle_position = !toggle_position;  // Flip the toggle for next time
}

// Lift Top Left Blue UP (Motor 11)
int lift_top_left_blue_up[NUM_MOTORS + 1] = {0, 
    1632, 2253, 2060, 2490, 1920, 983, 693, 2133, 1013, 2324, 1675, 1061
};

// Move Top Left Blue Forward (Motor 10)
int move_top_left_blue_fw[NUM_MOTORS + 1] = {0, 
    1632, 2253, 2059, 2491, 1922, 983, 693, 2133, 1014, 2549, 1759, 1060
};

// Set Motor 11 Down
int set_motor_11_down[NUM_MOTORS + 1] = {0, 
    1632, 2253, 2060, 2490, 1919, 983, 693, 2133, 1014, 2549, 1863, 1064
};



int main() 
{
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


  std::string input;

  while (true) {
    // Get user input
    std::cout << "Enter command: ";
    std::getline(std::cin, input);

    // Trim leading/trailing spaces
    if (input.empty()) continue;

    // Handle exit condition
    if (input == "exit") break;

    // Extract first word as command
    std::istringstream iss(input);
    std::string command;
    iss >> command;

    if (command == "get") {
      scan_motors(groupSyncRead, packetHandler, portHandler);
    }

    else if (command == "h1") {
      // int home_walking[NUM_MOTORS + 1] = {0, 
      //     2716, 2284, 2051, 1358, 1809, 1020, 2721, 2277, 1023, 2328, 1774, 1056
      // };
      // move_to(home_walking, groupSyncWrite, packetHandler,groupSyncRead, portHandler);
    
      int home_walking2[NUM_MOTORS + 1] = {0, 
          2879, 2032, 2059, 1440, 2066, 999, 2835, 2009, 1055, 2429, 2078, 1028
      };
      move_to(home_walking2, groupSyncWrite, packetHandler,groupSyncRead, portHandler);
    }

    else if (command == "fw") {
      // int home_walking[NUM_MOTORS + 1] = {0, 
      //     2716, 2284, 2051, 1358, 1809, 1020, 2721, 2277, 1023, 2328, 1774, 1056
      // };
      // move_to(home_walking, groupSyncWrite, packetHandler,groupSyncRead, portHandler); 

      int home_walking2[NUM_MOTORS + 1] = {0, 
          2879, 2032, 2059, 1440, 2066, 999, 2835, 2009, 1055, 2429, 2078, 1028
      };

      int leg_ids[4] = {4, 2, 3, 1};
      int up_degree = 15;
      int cw_degree = 15;

      while (1) 
      {
        int leg_num = 4;
        LegMotors motors = leg_motor_map[leg_num];
        present_positions[motors.roll_motor_id] = go_up(leg_num, up_degree);
        move_to(present_positions, groupSyncWrite, packetHandler,groupSyncRead, portHandler); 
        present_positions[motors.yaw_motor_id] = go_clockwise(leg_num, cw_degree);
        move_to(present_positions, groupSyncWrite, packetHandler,groupSyncRead, portHandler); 
        present_positions[motors.roll_motor_id] = go_down(leg_num, up_degree);
        move_to(present_positions, groupSyncWrite, packetHandler,groupSyncRead, portHandler);
        present_positions[motors.yaw_motor_id] = go_counter_clockwise(leg_num, cw_degree);
        move_to(present_positions, groupSyncWrite, packetHandler,groupSyncRead, portHandler);
        
        leg_num = 2;
        motors = leg_motor_map[leg_num];
        present_positions[motors.roll_motor_id] = go_up(leg_num, up_degree);
        move_to(present_positions, groupSyncWrite, packetHandler,groupSyncRead, portHandler); 
        present_positions[motors.yaw_motor_id] = go_counter_clockwise(leg_num, cw_degree);
        move_to(present_positions, groupSyncWrite, packetHandler,groupSyncRead, portHandler); 
        present_positions[motors.roll_motor_id] = go_down(leg_num, up_degree);
        move_to(present_positions, groupSyncWrite, packetHandler,groupSyncRead, portHandler);
        present_positions[motors.yaw_motor_id] = go_clockwise(leg_num, cw_degree);
        move_to(present_positions, groupSyncWrite, packetHandler,groupSyncRead, portHandler);

        leg_num = 3;
        motors = leg_motor_map[leg_num];
        present_positions[motors.roll_motor_id] = go_up(leg_num, up_degree);
        move_to(present_positions, groupSyncWrite, packetHandler,groupSyncRead, portHandler); 
        present_positions[motors.yaw_motor_id] = go_counter_clockwise(leg_num, cw_degree);
        move_to(present_positions, groupSyncWrite, packetHandler,groupSyncRead, portHandler); 
        present_positions[motors.roll_motor_id] = go_down(leg_num, up_degree);
        move_to(present_positions, groupSyncWrite, packetHandler,groupSyncRead, portHandler);
        present_positions[motors.yaw_motor_id] = go_clockwise(leg_num, cw_degree);
        move_to(present_positions, groupSyncWrite, packetHandler,groupSyncRead, portHandler);
        
        leg_num = 1;
        motors = leg_motor_map[leg_num];
        present_positions[motors.roll_motor_id] = go_up(leg_num, up_degree);
        move_to(present_positions, groupSyncWrite, packetHandler,groupSyncRead, portHandler); 
        present_positions[motors.yaw_motor_id] = go_clockwise(leg_num, cw_degree);
        move_to(present_positions, groupSyncWrite, packetHandler,groupSyncRead, portHandler); 
        present_positions[motors.roll_motor_id] = go_down(leg_num, up_degree);
        move_to(present_positions, groupSyncWrite, packetHandler,groupSyncRead, portHandler);
        present_positions[motors.yaw_motor_id] = go_counter_clockwise(leg_num, cw_degree);
        move_to(present_positions, groupSyncWrite, packetHandler,groupSyncRead, portHandler);
      }
    }

    else if (command == "up") {
      int degree;
      char colon;
      std::vector<int> leg_ids;

      if (!(iss >> degree >> colon) || colon != ':') {
        std::cout << "Invalid format. Expected 'up X:Y Z ...'\n";
        continue;
      }

      int leg_id;
      while (iss >> leg_id) {
        leg_ids.push_back(leg_id);
      }

      if (leg_ids.empty()) {
        std::cout << "Error: No leg IDs provided.\n";
      } else {
        std::cout << "Moving up " << degree << " degrees for IDs: ";
        for (int i : leg_ids) std::cout << i << " ";
        std::cout << std::endl;

        // WEIRD UNISOLATED MOVEMENT 
        // for (int leg_num : leg_ids) {
        //   LegMotors motors = leg_motor_map[leg_num];
        //   present_positions[motors.roll_motor_id] = go_up(leg_num, degree);
        // }
        // move_to(
        //   present_positions,
        //   groupSyncWrite, 
        //   packetHandler,
        //   groupSyncRead,
        //   portHandler);
        // Create a local copy of present positions
        int updated_positions[NUM_MOTORS + 1];
        std::copy(std::begin(present_positions), std::end(present_positions), std::begin(updated_positions));

        // Modify only roll motors
        for (int leg_num : leg_ids) {
            LegMotors motors = leg_motor_map[leg_num];
            updated_positions[motors.roll_motor_id] = go_up(leg_num, degree);
        }

        // Move only the modified motors
        move_to(updated_positions, groupSyncWrite, packetHandler, groupSyncRead, portHandler); 
      }
    }

    else if (command == "down") {
      int degree;
      char colon;
      std::vector<int> leg_ids;

      if (!(iss >> degree >> colon) || colon != ':') {
        std::cout << "Invalid format. Expected 'up X:Y Z ...'\n";
        continue;
      }

      int leg_id;
      while (iss >> leg_id) {
        leg_ids.push_back(leg_id);
      }

      if (leg_ids.empty()) {
        std::cout << "Error: No leg IDs provided.\n";
      } else {
        std::cout << "Moving down " << degree << " degrees for IDs: ";
        for (int i : leg_ids) std::cout << i << " ";
        std::cout << std::endl;

        // WEIRD UNISOLATED MOVEMENT 
        // for (int leg_num : leg_ids) {
        //   LegMotors motors = leg_motor_map[leg_num];
        //   present_positions[motors.roll_motor_id] = go_down(leg_num, degree);
        // }
        // move_to(
        //   present_positions,
        //   groupSyncWrite, 
        //   packetHandler,
        //   groupSyncRead,
        //   portHandler); 

        // Create a local copy of present positions
        int updated_positions[NUM_MOTORS + 1];
        std::copy(std::begin(present_positions), std::end(present_positions), std::begin(updated_positions));

        // Modify only roll motors
        for (int leg_num : leg_ids) {
            LegMotors motors = leg_motor_map[leg_num];
            updated_positions[motors.roll_motor_id] = go_down(leg_num, degree);
        }

        // Move only the modified motors
        move_to(updated_positions, groupSyncWrite, packetHandler, groupSyncRead, portHandler);
      }
    }

    else if (command == "cw") {
      int degree;
      char colon;
      std::vector<int> leg_ids;

      if (!(iss >> degree >> colon) || colon != ':') {
        std::cout << "Invalid format. Expected 'up X:Y Z ...'\n";
        continue;
      }

      int leg_id;
      while (iss >> leg_id) {
        leg_ids.push_back(leg_id);
      }

      if (leg_ids.empty()) {
        std::cout << "Error: No leg IDs provided.\n";
      } else {
        std::cout << "Moving clockwise " << degree << " degrees for IDs: ";
        for (int i : leg_ids) std::cout << i << " ";
        std::cout << std::endl;

        for (int leg_num : leg_ids) {
          LegMotors motors = leg_motor_map[leg_num];
          present_positions[motors.yaw_motor_id] = go_clockwise(leg_num, degree);
        }
        move_to(
          present_positions,
          groupSyncWrite, 
          packetHandler,
          groupSyncRead,
          portHandler); 
      }
    }

    else if (command == "ccw") {
      int degree;
      char colon;
      std::vector<int> leg_ids;

      if (!(iss >> degree >> colon) || colon != ':') {
        std::cout << "Invalid format. Expected 'up X:Y Z ...'\n";
        continue;
      }

      int leg_id;
      while (iss >> leg_id) {
        leg_ids.push_back(leg_id);
      }

      if (leg_ids.empty()) {
        std::cout << "Error: No leg IDs provided.\n";
      } else {
        std::cout << "Moving counter-clockwise " << degree << " degrees for IDs: ";
        for (int i : leg_ids) std::cout << i << " ";
        std::cout << std::endl;

        // move legs 1 by 1
        // for (int leg_num : leg_ids) {
        //   LegMotors motors = leg_motor_map[leg_num];

        //   present_positions[motors.yaw_motor_id] = go_counter_clockwise(leg_num, degree);
        //   move_to(
        //   present_positions,
        //   groupSyncWrite, 
        //   packetHandler,
        //   groupSyncRead,
        //   portHandler); 
        // }

        // move legs simul.
        for (int leg_num : leg_ids) {
          LegMotors motors = leg_motor_map[leg_num];
          present_positions[motors.yaw_motor_id] = go_counter_clockwise(leg_num, degree);
        }
        move_to(
          present_positions,
          groupSyncWrite, 
          packetHandler,
          groupSyncRead,
          portHandler); 
      }
    }
    

    else if (command == "en" || command == "d") {
      std::vector<int> ids;
      int id;
      while (iss >> id) {
        ids.push_back(id);
      }

      if (ids.empty()) {
        std::cout << "Error: No motor IDs provided.\n";
      } else {
        std::string ids_str;
        for (int id : ids) {
            ids_str += std::to_string(id) + " ";  // Convert vector to a space-separated string
        }

        // Create a mutable char array (dangerous but works)
        char ids_cstr[ids_str.length() + 1];
        strcpy(ids_cstr, ids_str.c_str());

        set_torque(packetHandler, portHandler, command.c_str(), ids_cstr);

      }
    }


    else if (command == "set") {
      std::unordered_map<int, int> positions;
      std::string pair;
      
      // Parse input format: "set ID:pos ID:pos ..."
      while (iss >> pair) {
          size_t pos = pair.find(':');
          if (pos == std::string::npos) {
              std::cout << "Invalid format. Expected 'set ID:pos ID:pos ...'\n";
              break;
          }
          int id = std::stoi(pair.substr(0, pos));
          int posValue = std::stoi(pair.substr(pos + 1));
          positions[id] = posValue;
      }

      if (!positions.empty()) {
        // Clear previous SyncWrite parameters
        groupSyncWrite.clearParam();

        for (const auto& [dxl_id, goal_position] : positions) {
            std::cout << "Moving Dynamixel ID " << dxl_id << " to Position " << goal_position << "\n";

            uint8_t param_goal_position[4];
            param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(goal_position));
            param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(goal_position));
            param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(goal_position));
            param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(goal_position));

            // Add to SyncWrite buffer
            if (!groupSyncWrite.addParam(dxl_id, param_goal_position)) {
                std::cerr << "[ID:" << dxl_id << "] groupSyncWrite addParam failed\n";
                continue;
            }
        }

        // Transmit goal positions to all motors at once
        dxl_comm_result = groupSyncWrite.txPacket();
        if (dxl_comm_result != COMM_SUCCESS) {
            std::cout << "SyncWrite Error: " << packetHandler->getTxRxResult(dxl_comm_result) << "\n";
        }

        // Clear SyncWrite buffer
        groupSyncWrite.clearParam();
      }
    }
    else if (command == "ali") {
      move_to(
        aligned_before_rolling,
        groupSyncWrite, 
        packetHandler,
        groupSyncRead,
        portHandler); 
    }
    else {
      std::cout << "Unknown command: " << command << "\n";
    }
  }

  // for (int i = 0; i < 20; i++) {
  //   // Disable Dynamixel# Torque
  //   dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
  //   if (dxl_comm_result != COMM_SUCCESS)
  //   {
  //     printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  //   }
  //   else if (dxl_error != 0)
  //   {
  //     printf("%s\n", packetHandler->getRxPacketError(dxl_error));
  //   }
  // }

// Close port
portHandler->closePort();

return 0;

}