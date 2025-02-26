#ifndef QUAD_MOTOR_CONTROL_HPP_
#define QUAD_MOTOR_CONTROL_HPP_

#include <cstdio>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "quad_interfaces/msg/set_position.hpp"
#include "quad_interfaces/srv/get_position.hpp"

class QuadMotorControl : public rclcpp::Node 
{
public:
    using SetPosition = quad_interfaces::msg::SetPosition;
    using GetPosition = quad_interfaces::srv::GetPosition;

    QuadMotorControl();
    virtual ~QuadMotorControl();

private:
    dynamixel::PortHandler * portHandler;
    dynamixel::PacketHandler * packetHandler;
    dynamixel::GroupSyncWrite* groupSyncWrite;

    void initDynamixels();

    rclcpp::Subscription<SetPosition>::SharedPtr set_position_subscriber_;
    rclcpp::Service<GetPosition>::SharedPtr get_position_server_;

    int present_position;
};

#endif  // QUAD_MOTOR_CONTROL_HPP_