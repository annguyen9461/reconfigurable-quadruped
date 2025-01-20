

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

#if defined(__linux__) || defined(__APPLE__)
#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

#include <stdlib.h>
#include <stdio.h>
#include "dynamixel_sdk.h"  // Uses DYNAMIXEL SDK library

/********* DYNAMIXEL Model definition *********/
#define X_SERIES // X330, X430, X540, 2X430

// Control table address
#if defined(X_SERIES) || defined(MX_SERIES)
  #define ADDR_TORQUE_ENABLE          64
  #define ADDR_GOAL_POSITION          116
  #define ADDR_PRESENT_POSITION       132
  #define BAUDRATE                    57600
#endif

// DYNAMIXEL Protocol Version
#define PROTOCOL_VERSION  2.0

#define ADDR_GOAL_POSITION1 200
#define ADDR_GOAL_POSITION2 200
#define ADDR_GOAL_POSITION3 200

#define ADDR_PRESENT_POSITION1 200
#define ADDR_PRESENT_POSITION2 200
#define ADDR_PRESENT_POSITION3 200

// Motors attached to body
// Motor IDs: 1, 4, 7, 10
#define MIN_POSITION_LIMIT_MOTOR1      135  
#define MAX_POSITION_LIMIT_MOTOR1      315

// Motors moving leg up
// Motor IDs: 2, 5, 8, 11
#define MIN_POSITION_LIMIT_MOTOR2      223      // UP
#define MAX_POSITION_LIMIT_MOTOR2      250      // DOWN

// Motors folding blades
// Motor IDs: 3, 6, 9, 12
#define MIN_POSITION_LIMIT_MOTOR3      153  
#define MAX_POSITION_LIMIT_MOTOR3      215  

// DYNAMIXEL IDs

// KEEP THIS TO INITIALIZEE
#define MOTOR1_ID  20
#define MOTOR2_ID  20
#define MOTOR3_ID  20

// CHANGE THIS
#define MOTOR1_ID  4
#define MOTOR2_ID  20
#define MOTOR3_ID  20

// Port configuration
#define DEVICENAME  "/dev/ttyUSB1"

#define TORQUE_ENABLE                   1
#define TORQUE_DISABLE                  0
#define DXL_MOVING_STATUS_THRESHOLD     20
#define ESC_ASCII_VALUE                 0x1b

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

int kbhit(void) {
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
  if (ch != EOF) {
    ungetc(ch, stdin);
    return 1;
  }
  return 0;
#elif defined(_WIN32) || defined(_WIN64)
  return _kbhit();
#endif
}

int main() {
  // Initialize PortHandler instance
  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

  // Initialize PacketHandler instance
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  int dxl_comm_result = COMM_TX_FAIL;
  int dxl1_goal_position[2] = {MIN_POSITION_LIMIT_MOTOR1, MAX_POSITION_LIMIT_MOTOR1};
  int dxl2_goal_position[2] = {MIN_POSITION_LIMIT_MOTOR2, MAX_POSITION_LIMIT_MOTOR2};
  int dxl3_goal_position[2] = {MIN_POSITION_LIMIT_MOTOR3, MAX_POSITION_LIMIT_MOTOR3};

  uint8_t dxl_error = 0;
  // TODO: CHANGE PRESENT POSITION AS NEEDED
  int32_t dxl1_present_position = 153;
  int32_t dxl2_present_position = 315;
  int32_t dxl3_present_position = 333;

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

  // Enable Torque for both servos
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, MOTOR1_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  } else if (dxl_error != 0) {
    printf("%s\n", packetHandler->getRxPacketError(dxl_error));
  }

  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, MOTOR2_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  } else if (dxl_error != 0) {
    printf("%s\n", packetHandler->getRxPacketError(dxl_error));
  }

  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, MOTOR3_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  } else if (dxl_error != 0) {
    printf("%s\n", packetHandler->getRxPacketError(dxl_error));
  }

  int index = 0;
  while (1) {
    printf("Press any key to continue. (Press [ESC] to exit)\n");
    if (getch() == ESC_ASCII_VALUE) break;

    // Write goal positions for both servos
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, MOTOR1_ID, ADDR_GOAL_POSITION1, dxl1_goal_position[index], &dxl_error);
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, MOTOR2_ID, ADDR_GOAL_POSITION2, dxl2_goal_position[index], &dxl_error);
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, MOTOR3_ID, ADDR_GOAL_POSITION3, dxl3_goal_position[index], &dxl_error);

    do {
      // Read present positions
      dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, MOTOR1_ID, ADDR_PRESENT_POSITION1, (uint32_t*)&dxl1_present_position, &dxl_error);
      dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, MOTOR2_ID, ADDR_PRESENT_POSITION2, (uint32_t*)&dxl2_present_position, &dxl_error);
      dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, MOTOR3_ID, ADDR_PRESENT_POSITION3, (uint32_t*)&dxl2_present_position, &dxl_error);

      printf("[ID:%03d] GoalPos:%03d  PresPos:%03d [ID:%03d] GoalPos:%03d  PresPos:%03d\n",
              MOTOR1_ID, dxl1_goal_position[index], dxl1_present_position,
              MOTOR2_ID, dxl2_goal_position[index], dxl2_present_position,
              MOTOR3_ID, dxl3_goal_position[index], dxl3_present_position);
    } while ((abs(dxl1_goal_position[index] - dxl1_present_position) > DXL_MOVING_STATUS_THRESHOLD) 
              ||  (abs(dxl2_goal_position[index] - dxl2_present_position) > DXL_MOVING_STATUS_THRESHOLD)
              ||  (abs(dxl3_goal_position[index] - dxl3_present_position) > DXL_MOVING_STATUS_THRESHOLD)
              );

    index = (index == 0) ? 1 : 0;
  }

  // Disable Torque
  packetHandler->write1ByteTxRx(portHandler, MOTOR1_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
  packetHandler->write1ByteTxRx(portHandler, MOTOR2_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
  packetHandler->write1ByteTxRx(portHandler, MOTOR3_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);

  // Close port
  portHandler->closePort();
  return 0;
}