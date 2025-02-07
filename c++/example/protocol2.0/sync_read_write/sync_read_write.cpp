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

#define NUM_MOTORS                      20                  // IDs from 1 to 12


int DXL_ID;
bool toggle_position = false;  // Toggles between the two positions
bool forward_running = false;

void scan_motors(dynamixel::GroupSyncRead &groupSyncRead, 
                 dynamixel::PacketHandler *packetHandler, 
                 dynamixel::PortHandler *portHandler);

void set_torque(dynamixel::PacketHandler *packetHandler, 
                dynamixel::PortHandler *portHandler, 
                const char *command, char *ids_str); 

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
  for (int id = 1; id <= MAX_ID; id++) {
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
  for (int id = 1; id <= MAX_ID; id++) {
      if (groupSyncRead.isAvailable(id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)) {
          int32_t position = groupSyncRead.getData(id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
          printf("[ID:%d] Position: %d\n", id, position);
      }
  }
  printf("\n");
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

int home_positions[NUM_MOTORS + 1] = {0, 
  1289, 2051, 2062, 2908, 2046, 990, 342, 1987, 1000, 2833, 2069, 1060
};
int circle_positions[NUM_MOTORS + 1] = {0, 
  1062, 2947, 24, 3059, 1174, 3046, 22, 1096, 1004, 3089, 2943, 1055
};

void move_to(
          int* positions,
          dynamixel::GroupSyncWrite &groupSyncWrite, 
          dynamixel::PacketHandler *packetHandler) 
{
  printf("Moving all motors to home position...\n");

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

  printf("All motors moved to home position.\n");
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


  while (1)
  {
    char input[MAX_INPUT_SIZE];

    printf("Enter motor ID and position (e.g., '14:1000, 15:2000') type 'exit' to quit:\n");
    fgets(input, sizeof(input), stdin);   // Read user input
    input[strcspn(input, "\n")] = 0;  // Remove newline character

    // Exit condition
    if (strncmp(input, "exit", 4) == 0) break;

    // Parse first word
    char *command = strtok(input, " ");
    char *args = strtok(NULL, "");  // Get the rest of the input

    if (command == NULL) continue;  // Skip empty input

    // If user enters "get", scan all Dynamixel IDs
    if (strcmp(command, "get") == 0) {
      scan_motors(groupSyncRead, packetHandler, portHandler);
    }

    // If user enters "home", move all motors to home positions
    else if (strcmp(command, "ho") == 0) {
      move_to(home_positions, groupSyncWrite, packetHandler);
    }

    // If user enters "home", move all motors to home positions
    else if (strcmp(command, "ci") == 0) {
      move_to(circle_positions, groupSyncWrite, packetHandler);
    }

    // MOVE FORWARD
    else if (strcmp(command, "fw") == 0) {
        if (!forward_running) {
          move_forward(groupSyncWrite, packetHandler);
        } else {
          printf("Already moving forward! Type 'stop' to halt.\n");
        }
    }
    // MOVE FORWARD CONTINUOUSLY
    else if (strcmp(command, "fwc") == 0) {
        if (!forward_running) {
          while (1) {
            move_forward(groupSyncWrite, packetHandler);
          }
        } else {
          printf("Already moving forward! Type 'stop' to halt.\n");
        }
    }

    // If user enters "stop", halt continuous movement
    else if (strcmp(command, "stop") == 0) {
        if (forward_running) {
            forward_running = false;
            printf("Stopping forward motion...\n");
        } else {
            printf("Motors are not moving.\n");
        }
    }

    // handle "enable" or "disable" command
    else if (strcmp(command, "en") == 0 || strcmp(command, "d") == 0)
    {
      if (args == NULL) {
        printf("Error: No motor IDs provides.\n");
      } else {
        set_torque(packetHandler, portHandler, command, args);
      }
    }
    // handle setting motor positions
    else
    {
      int dxl_id, goal_position;
      char *token = strtok(input, ", ");  // Split by comma and space

      // Clear previous SyncWrite parameters
      groupSyncWrite.clearParam();

      while (token != NULL) 
      {
          // Find the colon separator
          char *colon = strchr(token, ':');
          if (!colon) {
              printf("Invalid format. Expected: ID1:Position1, ID2:Position2\n");
              break;
          }

          // Extract ID and position
          *colon = '\0';    // Replace ':' with null to split key and value
          dxl_id = atoi(token);   // Convert ID
          goal_position = atoi(colon + 1);   // Convert position

          printf("Moving Dynamixel ID %d to Position %d\n", dxl_id, goal_position);

          // Send goal position to the motor
          uint8_t param_goal_position[4];
          param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(goal_position));
          param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(goal_position));
          param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(goal_position));
          param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(goal_position));

          // Add to SyncWrite buffer
          if (!groupSyncWrite.addParam(dxl_id, param_goal_position)) {
              fprintf(stderr, "[ID:%03d] groupSyncWrite addParam failed\n", dxl_id);
              continue;
          }

          token = strtok(NULL, ", ");  // Move to next pair
      }

      // Transmit goal positions to all motors at once
      dxl_comm_result = groupSyncWrite.txPacket();
      if (dxl_comm_result != COMM_SUCCESS) {
          printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
      }

      // Clear SyncWrite buffer
      groupSyncWrite.clearParam();
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
