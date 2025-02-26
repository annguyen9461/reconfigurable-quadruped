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

#include "position_configs.hpp"
// #include <vector>

class QuadMotorControl : public rclcpp::Node 
{
public:
    using SetPosition = quad_interfaces::msg::SetPosition;
    using SetConfig = quad_interfaces::msg::SetConfig;
    using GetPosition = quad_interfaces::srv::GetPosition;
    using GetAllPositions = quad_interfaces::srv::GetAllPositions;

    QuadMotorControl();
    virtual ~QuadMotorControl();

private:
    dynamixel::PortHandler* portHandler;
    dynamixel::PacketHandler* packetHandler;
    dynamixel::GroupSyncWrite* groupSyncWrite;
    dynamixel::GroupSyncRead* groupSyncRead;

    void initDynamixels();

    rclcpp::Subscription<SetPosition>::SharedPtr set_position_subscriber_;
    rclcpp::Subscription<SetConfig>::SharedPtr set_config_subscriber_;
    rclcpp::Service<GetPosition>::SharedPtr get_position_server_;
    rclcpp::Service<GetAllPositions>::SharedPtr get_all_positions_server_;

    int present_position;
    // std::vector<int> all_positions  = {0, 
    //     2745,  // [ID:1]
    //     2187,  // [ID:2]
    //     3062,  // [ID:3]
    //     1343,  // [ID:4]
    //     1890,  // [ID:5]
    //     1025,  // [ID:6]
    //     2752,  // [ID:7]
    //     2190,  // [ID:8]
    //     3072,  // [ID:9]
    //     2429,  // [ID:10]
    //     1864,  // [ID:11]
    //     1050   // [ID:12]
    // };
};

#endif  // QUAD_MOTOR_CONTROL_HPP_