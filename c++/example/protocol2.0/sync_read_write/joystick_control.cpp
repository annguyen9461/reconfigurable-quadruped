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

#include "joystick.hpp"
#include <unordered_set>

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

// 1 for WALKING, 2 for ROLLING
int mode = 1;
int present_positions[NUM_MOTORS + 1] = {0, 
  2045, 2053, 3049, 2054, 2035, 1014, 2044, 2047, 3071, 3051, 2043, 1056
};

int aligned_before_rolling[NUM_MOTORS + 1] = {0, 
  2045, 2053, 3049, 2054, 2035, 1014, 2044, 2047, 3071, 3051, 2043, 1056
};
int home_walking2[NUM_MOTORS + 1] = {0, 
  2879, 2032, 3049, 1440, 2066, 1014, 2835, 2009, 3071, 2429, 2078, 1056
};

int home_tiptoe[NUM_MOTORS + 1] = {0, 
  2882, 2127, 3046, 1438, 1951, 1015, 2836, 2134, 3069, 2429, 1957, 1058
};
int home_tiptoe_thin[NUM_MOTORS + 1] = {0, 
  2207, 2325, 3053, 1818, 1789, 1020, 2226, 2299, 3070, 2833, 1786, 1049
};

int perfect_cir[NUM_MOTORS + 1] = {0, 
  2039, 1113, 3080, 2053, 2980, 1006, 2086, 2983, 1045, 3054, 1112, 3094
};

// Up and Down movement (2, 5, 8, 11) (ROLL)
// WALKING
// #define DOWN_MOTOR2  2048  // Down (flat no under)
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

// WALKING
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

// ROLLING
// Struct to store motors for rolling
struct LegMotorsFold {
  int fold_motor_id;
  int fold_cw;
  int fold_ccw;
};

// Map each leg number to its corresponding motors (using defines)
std::unordered_map<int, LegMotorsFold> fold_map = {
  {1, {3, CLOCKWISE_MOTOR3, COUNTER_CLOCKWISE_MOTOR3}},  
  {2, {6, CLOCKWISE_MOTOR6, COUNTER_CLOCKWISE_MOTOR6}},  
  {3, {9, CLOCKWISE_MOTOR9, COUNTER_CLOCKWISE_MOTOR9}},  
  {4, {12, CLOCKWISE_MOTOR12, COUNTER_CLOCKWISE_MOTOR12}}
};


int degree_to_pos_diff(int degree) {
  return static_cast<int>((degree/360.0) * 4095);   // used 360.0 to prevent zero for small angles
}

// Fold the motor CW by a given degree amount
int fold_cw(int leg_num, int degree) {
  LegMotorsFold motors = fold_map[leg_num];
  int fold_cw = motors.fold_cw;     // Clockwise position
  int fold_ccw = motors.fold_ccw;   // Counter-clockwise position
  int diff = degree_to_pos_diff(degree);
  int curr_pos_motor = present_positions[motors.fold_motor_id];

  // Ensure position stays within limits when moving CLOCKWISE
  if (fold_ccw > fold_cw) {
      return std::max(curr_pos_motor - diff, fold_cw);
  } else {
      return std::min(curr_pos_motor + diff, fold_cw);
  }
}

// Fold the motor CCW by a given degree amount
int fold_ccw(int leg_num, int degree) {
  LegMotorsFold motors = fold_map[leg_num];
  int fold_cw = motors.fold_cw;     // Clockwise position
  int fold_ccw = motors.fold_ccw;   // Counter-clockwise position
  int diff = degree_to_pos_diff(degree);
  int curr_pos_motor = present_positions[motors.fold_motor_id];

  // Ensure position stays within limits when moving CLOCKWISE
  if (fold_ccw > fold_cw) {
      return std::max(curr_pos_motor + diff, fold_cw);
  } else {
      return std::min(curr_pos_motor - diff, fold_cw);
  }
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
}

void update_present_positions_rolling(dynamixel::GroupSyncRead &groupSyncRead, 
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

    std::unordered_set<int> rolling_motors {2, 5, 8, 11};
   
    // Update present positions of connected motors
    for (int id = 1; id <= NUM_MOTORS; id++) {
        if (groupSyncRead.isAvailable(id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)) {
            if (rolling_motors.find(id) == rolling_motors.end()) {   
                // don't update those NON-ROLLING motors
                present_positions[id] = perfect_cir[id];
            } else {    // update rolling motors
                int32_t position = groupSyncRead.getData(id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
                present_positions[id] = position;
            }

        }
    }
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
  std::this_thread::sleep_for(std::chrono::milliseconds(10));  // Allow TIME for motors to reach the position
  update_present_positions(groupSyncRead, packetHandler, portHandler);

  // Print the updated positions for debugging
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


void gradual_transition(int* next_positions, 
                        dynamixel::GroupSyncWrite &groupSyncWrite, 
                        dynamixel::PacketHandler *packetHandler) {
    const int step_size = 60;  // number of steps for smooth transition
    int step_arr[NUM_MOTORS + 1] = {0};
    int num_motors = NUM_MOTORS;
    // loop range to start from 1 (ignoring index 0)

    int updated_positions[NUM_MOTORS + 1];
    std::copy(std::begin(present_positions), std::end(present_positions), std::begin(updated_positions));

    for (int i = 1; i <= num_motors; i++) {
        step_arr[i] = (next_positions[i] - updated_positions[i]) / step_size;
    }
    // perform transitions for each step
    for (int step = 0; step < step_size; step++) {
        // update motor transition based on respective step size
        for (int i = 1; i <= num_motors; i++) {
            updated_positions[i] += step_arr[i];
        }
        move_to_target_positions(updated_positions, groupSyncWrite, packetHandler);
    }
    // ensure final position is accurate (due to integer division)
    move_to_target_positions(next_positions, groupSyncWrite, packetHandler);
}

int main() 
{
    int num_legs = 4;
    std::unordered_set<int> btn_set;
    std::unordered_set<int> leg_set;

    const unsigned int maxJoysticks = 32;
    Joystick joysticks[maxJoysticks] = {0};

    char fileName[32];
    for (unsigned int i=0; i<maxJoysticks; ++i)
    {
        sprintf(fileName, "/dev/input/js%d", i);
        joysticks[i] = openJoystick(fileName);
    }

    bool going_up = 0;
    bool going_down = 0;
    bool is_flat = 1;

    bool move_yellow = 1;
    leg_set.insert(1);
    leg_set.insert(2);
    leg_set.erase(3);
    leg_set.erase(4);  

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

    int js_command = 99;
    int btn_command = 99;
    while (true) {
        for (unsigned int i=0; i<maxJoysticks; ++i)
        {
        if (joysticks[i].connected)
        {
            readJoystickInput(&joysticks[i]);

            printf("Axes: ");
                    int val_updown = joysticks[i].axisStates[RIGHT_JS_UP_DOWN];
                    int val_left_right = joysticks[i].axisStates[RIGHT_JS_LEFT_RIGHT];

                    printf("%d:% 6d ", RIGHT_JS_LEFT_RIGHT, val_left_right);
                    printf("%d:% 6d ", RIGHT_JS_UP_DOWN, val_updown);
                    
            printf("Button pressed: ");
            for (char buttonIndex=0; buttonIndex<joysticks[i].buttonCount; ++buttonIndex) {
            // if pressed
            if (joysticks[i].buttonStates[buttonIndex]) {   
                    printf("%d ", buttonIndex);
                    if (buttonIndex <= 3) {
                        if (btn_set.find(buttonIndex) == btn_set.end()) { // Add the button if not present
                            btn_set.insert(buttonIndex);
                            leg_set.insert(buttonIndex+1);
                        } else {
                            btn_set.erase(buttonIndex);
                            leg_set.erase(buttonIndex+1);
                        }
                    }
                }
            }
            printf("\n");

            printf("Button list: ");
            for (auto btn : btn_set) {
                printf("%d, ",btn);
            }
            printf("\n");
            printf("Leg list: ");
            for (auto leg : leg_set) {
                printf("%d, ",leg);
            }
            printf("\n");

            if (val_updown == 0 && val_left_right == 0 ) {
                going_up = 0;
                going_down = 0;
                is_flat = 1;
            }
                else {
                if (abs(val_updown) > abs(val_left_right)) {
                    if (val_updown < 0) 
                    {
                        printf("UP\n");
                        going_up = 1;
                        going_down = 0;
                        is_flat = 0;
                    } else if(val_updown > 0) 
                    {
                        printf("DOWN\n");
                        going_up = 0;
                        going_down = 1;
                        is_flat = 0;
                    }
                } else {
                    if (val_left_right < 0) {
                        printf("LEFT\n");
                    } 
                    else if (val_left_right > 0)
                    {
                        printf("RIGHT\n");
                    }
                }
            }

            if (going_up) {
                int degree = 1;

                std::cout << "Moving up " << degree << " degrees for IDs: ";
                for (auto i : leg_set) std::cout << i << " ";
                std::cout << std::endl;

                // refresh present_positions with real motor values
                update_present_positions_rolling(groupSyncRead, packetHandler, portHandler);

                // modify only roll motors
                for (auto leg_num : leg_set) {
                    if (leg_num == 1 || leg_num == 2) {
                        LegMotors motors = leg_motor_map[leg_num];
                        present_positions[motors.roll_motor_id] = go_up(leg_num, degree);
                    }
                    else {
                        LegMotors motors = leg_motor_map[leg_num];
                        present_positions[motors.roll_motor_id] = go_down(leg_num, degree);
                    }
                }

                // move only the modified motors
                gradual_transition(present_positions, groupSyncWrite, packetHandler);
            }
            else if (going_down) {
                int degree = 1;

                std::cout << "Moving down " << degree << " degrees for IDs: ";
                for (auto i : leg_set) std::cout << i << " ";
                std::cout << std::endl;

                // refresh present_positions with real motor values
                update_present_positions_rolling(groupSyncRead, packetHandler, portHandler);

                // modify only roll motors
                for (auto leg_num : leg_set) {
                    if (leg_num == 1 || leg_num == 2) {
                        LegMotors motors = leg_motor_map[leg_num];
                        present_positions[motors.roll_motor_id] = go_down(leg_num, degree);
                    }
                    else {
                        LegMotors motors = leg_motor_map[leg_num];
                        present_positions[motors.roll_motor_id] = go_up(leg_num, degree);
                    }
                }

                // move only the modified motors
                gradual_transition(present_positions, groupSyncWrite, packetHandler);
            }
            else if (is_flat) {
                if (joysticks[i].buttonStates[BTN_START]) {  
                    move_to_target_positions(perfect_cir, groupSyncWrite, packetHandler);
                }
                else if (joysticks[i].buttonStates[BTN_SELECT]){
                    if (move_yellow) {
                        move_yellow = 0;
                        std::cout << "Moving Blue";
                        leg_set.insert(4);
                        leg_set.insert(3);
                        leg_set.erase(2);
                        leg_set.erase(1); 
                    } else {
                        move_yellow = 1;
                        std::cout << "Moving Yellow";
                        leg_set.insert(1);
                        leg_set.insert(2);
                        leg_set.erase(3);
                        leg_set.erase(4);  
                    }
                }
                else {
                    std::cout << "Unknown command: " << btn_command << "\n";
                }
            }
        }
    }
    
  }

// Close port
portHandler->closePort();

return 0;

}
