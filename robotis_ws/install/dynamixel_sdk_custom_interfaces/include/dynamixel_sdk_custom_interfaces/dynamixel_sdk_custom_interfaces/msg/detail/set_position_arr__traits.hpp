// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from dynamixel_sdk_custom_interfaces:msg/SetPositionArr.idl
// generated code does not contain a copyright notice

#ifndef DYNAMIXEL_SDK_CUSTOM_INTERFACES__MSG__DETAIL__SET_POSITION_ARR__TRAITS_HPP_
#define DYNAMIXEL_SDK_CUSTOM_INTERFACES__MSG__DETAIL__SET_POSITION_ARR__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "dynamixel_sdk_custom_interfaces/msg/detail/set_position_arr__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace dynamixel_sdk_custom_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const SetPositionArr & msg,
  std::ostream & out)
{
  out << "{";
  // member: ids
  {
    if (msg.ids.size() == 0) {
      out << "ids: []";
    } else {
      out << "ids: [";
      size_t pending_items = msg.ids.size();
      for (auto item : msg.ids) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: positions
  {
    if (msg.positions.size() == 0) {
      out << "positions: []";
    } else {
      out << "positions: [";
      size_t pending_items = msg.positions.size();
      for (auto item : msg.positions) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetPositionArr & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: ids
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.ids.size() == 0) {
      out << "ids: []\n";
    } else {
      out << "ids:\n";
      for (auto item : msg.ids) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: positions
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.positions.size() == 0) {
      out << "positions: []\n";
    } else {
      out << "positions:\n";
      for (auto item : msg.positions) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetPositionArr & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace dynamixel_sdk_custom_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use dynamixel_sdk_custom_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const dynamixel_sdk_custom_interfaces::msg::SetPositionArr & msg,
  std::ostream & out, size_t indentation = 0)
{
  dynamixel_sdk_custom_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use dynamixel_sdk_custom_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const dynamixel_sdk_custom_interfaces::msg::SetPositionArr & msg)
{
  return dynamixel_sdk_custom_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<dynamixel_sdk_custom_interfaces::msg::SetPositionArr>()
{
  return "dynamixel_sdk_custom_interfaces::msg::SetPositionArr";
}

template<>
inline const char * name<dynamixel_sdk_custom_interfaces::msg::SetPositionArr>()
{
  return "dynamixel_sdk_custom_interfaces/msg/SetPositionArr";
}

template<>
struct has_fixed_size<dynamixel_sdk_custom_interfaces::msg::SetPositionArr>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<dynamixel_sdk_custom_interfaces::msg::SetPositionArr>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<dynamixel_sdk_custom_interfaces::msg::SetPositionArr>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // DYNAMIXEL_SDK_CUSTOM_INTERFACES__MSG__DETAIL__SET_POSITION_ARR__TRAITS_HPP_
