// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dynamixel_sdk_custom_interfaces:msg/SetPositionArr.idl
// generated code does not contain a copyright notice

#ifndef DYNAMIXEL_SDK_CUSTOM_INTERFACES__MSG__DETAIL__SET_POSITION_ARR__BUILDER_HPP_
#define DYNAMIXEL_SDK_CUSTOM_INTERFACES__MSG__DETAIL__SET_POSITION_ARR__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dynamixel_sdk_custom_interfaces/msg/detail/set_position_arr__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dynamixel_sdk_custom_interfaces
{

namespace msg
{

namespace builder
{

class Init_SetPositionArr_positions
{
public:
  explicit Init_SetPositionArr_positions(::dynamixel_sdk_custom_interfaces::msg::SetPositionArr & msg)
  : msg_(msg)
  {}
  ::dynamixel_sdk_custom_interfaces::msg::SetPositionArr positions(::dynamixel_sdk_custom_interfaces::msg::SetPositionArr::_positions_type arg)
  {
    msg_.positions = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dynamixel_sdk_custom_interfaces::msg::SetPositionArr msg_;
};

class Init_SetPositionArr_ids
{
public:
  Init_SetPositionArr_ids()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetPositionArr_positions ids(::dynamixel_sdk_custom_interfaces::msg::SetPositionArr::_ids_type arg)
  {
    msg_.ids = std::move(arg);
    return Init_SetPositionArr_positions(msg_);
  }

private:
  ::dynamixel_sdk_custom_interfaces::msg::SetPositionArr msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::dynamixel_sdk_custom_interfaces::msg::SetPositionArr>()
{
  return dynamixel_sdk_custom_interfaces::msg::builder::Init_SetPositionArr_ids();
}

}  // namespace dynamixel_sdk_custom_interfaces

#endif  // DYNAMIXEL_SDK_CUSTOM_INTERFACES__MSG__DETAIL__SET_POSITION_ARR__BUILDER_HPP_
