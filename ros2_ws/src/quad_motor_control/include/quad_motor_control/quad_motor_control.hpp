#ifndef QUAD_MOTOR_CONTROL_HPP_
#define QUAD_MOTOR_CONTROL_HPP_

#include <cstdio>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "dynamixel_sdk/dynamixel_sdk.h"

class QuadMotorControl : public rclcpp::Node 
{
public:
    QuadMotorControl();
    virtual ~QuadMotorControl();
};

#endif  // QUAD_MOTOR_CONTROL_HPP_