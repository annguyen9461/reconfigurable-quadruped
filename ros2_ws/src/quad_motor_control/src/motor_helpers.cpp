#include "quad_motor_control/quad_motor_control.hpp"
#include <iostream>
#include <vector>
#include <thread>
#include <chrono>

// Move motors to target positions
void moveto(
    int* target_positions,
    dynamixel::GroupSyncWrite* groupSyncWrite,
    dynamixel::PacketHandler* packetHandler
)
{
    std::cout << "Moving motors..." << std::endl;

    // Ensure groupSyncWrite is valid before using it
    if (!groupSyncWrite) {
        std::cerr << "groupSyncWrite is not initialized!" << std::endl;
        return;
    }

    // Clear previous SyncWrite parameters
    groupSyncWrite->clearParam();

    for (int id = 1; id <= 12; id++)  // Loop through motor IDs 1-12
    {
        uint8_t param_goal_position[4];
        int goal_position = target_positions[id];

        param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(goal_position));
        param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(goal_position));
        param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(goal_position));
        param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(goal_position));

        // Add goal position to SyncWrite buffer
        if (!groupSyncWrite->addParam(id, param_goal_position)) {
            std::cerr << "[ID:" << id << "] groupSyncWrite addParam failed" << std::endl;
            continue;
        }
    }

    // Transmit the target positions to all motors at once
    int dxl_comm_result = groupSyncWrite->txPacket();
    if (dxl_comm_result != COMM_SUCCESS) {
        std::cerr << "Dynamixel Error: " << packetHandler->getTxRxResult(dxl_comm_result) << std::endl;
    }

    // Clear SyncWrite buffer after sending data
    groupSyncWrite->clearParam();

    std::cout << "Motors moved to target position." << std::endl;
}