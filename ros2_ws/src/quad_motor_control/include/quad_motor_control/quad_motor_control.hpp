#ifndef QUAD_MOTOR_CONTROL_HPP_
#define QUAD_MOTOR_CONTROL_HPP_

#include <cstdio>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "quad_interfaces/msg/set_position.hpp"
#include "quad_interfaces/msg/set_config.hpp"
#include "quad_interfaces/srv/get_position.hpp"
#include "quad_interfaces/srv/get_all_positions.hpp"

#include "quad_interfaces/msg/motor_positions.hpp"

#include "position_configs.hpp"
// #include <vector>

class QuadMotorControl : public rclcpp::Node 
{
public:
    using SetPosition = quad_interfaces::msg::SetPosition;
    using SetConfig = quad_interfaces::msg::SetConfig;
    using GetPosition = quad_interfaces::srv::GetPosition;
    using GetAllPositions = quad_interfaces::srv::GetAllPositions;

    using MotorPositions = quad_interfaces::msg::MotorPositions;

    QuadMotorControl();
    virtual ~QuadMotorControl();

private:
    dynamixel::PortHandler* portHandler;
    dynamixel::PacketHandler* packetHandler;
    dynamixel::GroupSyncWrite* groupSyncWrite;
    dynamixel::GroupSyncRead* groupSyncRead;

    rclcpp::Subscription<SetPosition>::SharedPtr set_position_subscriber_;
    rclcpp::Subscription<SetConfig>::SharedPtr set_config_subscriber_;
    rclcpp::Service<GetPosition>::SharedPtr get_position_server_;
    rclcpp::Service<GetAllPositions>::SharedPtr get_all_positions_server_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<quad_interfaces::msg::MotorPositions>::SharedPtr motor_positions_publisher_;

    // Helper functions
    void initDynamixels();
    void publishMotorPositions();
    void executeConfiguration(const SetConfig::SharedPtr msg);
    int readMotorPosition(int motor_id);

    int present_position;
};

// Function to move motors to a target position
void moveto(
    int* target_positions,
    dynamixel::GroupSyncWrite* groupSyncWrite,
    dynamixel::PacketHandler* packetHandler
);

#endif  // QUAD_MOTOR_CONTROL_HPP_