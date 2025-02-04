// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from dynamixel_sdk_custom_interfaces:msg/SetPositionArray.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "dynamixel_sdk_custom_interfaces/msg/detail/set_position_array__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace dynamixel_sdk_custom_interfaces
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void SetPositionArray_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) dynamixel_sdk_custom_interfaces::msg::SetPositionArray(_init);
}

void SetPositionArray_fini_function(void * message_memory)
{
  auto typed_message = static_cast<dynamixel_sdk_custom_interfaces::msg::SetPositionArray *>(message_memory);
  typed_message->~SetPositionArray();
}

size_t size_function__SetPositionArray__positions(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<dynamixel_sdk_custom_interfaces::msg::SetPosition> *>(untyped_member);
  return member->size();
}

const void * get_const_function__SetPositionArray__positions(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<dynamixel_sdk_custom_interfaces::msg::SetPosition> *>(untyped_member);
  return &member[index];
}

void * get_function__SetPositionArray__positions(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<dynamixel_sdk_custom_interfaces::msg::SetPosition> *>(untyped_member);
  return &member[index];
}

void fetch_function__SetPositionArray__positions(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const dynamixel_sdk_custom_interfaces::msg::SetPosition *>(
    get_const_function__SetPositionArray__positions(untyped_member, index));
  auto & value = *reinterpret_cast<dynamixel_sdk_custom_interfaces::msg::SetPosition *>(untyped_value);
  value = item;
}

void assign_function__SetPositionArray__positions(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<dynamixel_sdk_custom_interfaces::msg::SetPosition *>(
    get_function__SetPositionArray__positions(untyped_member, index));
  const auto & value = *reinterpret_cast<const dynamixel_sdk_custom_interfaces::msg::SetPosition *>(untyped_value);
  item = value;
}

void resize_function__SetPositionArray__positions(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<dynamixel_sdk_custom_interfaces::msg::SetPosition> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember SetPositionArray_message_member_array[1] = {
  {
    "positions",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<dynamixel_sdk_custom_interfaces::msg::SetPosition>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dynamixel_sdk_custom_interfaces::msg::SetPositionArray, positions),  // bytes offset in struct
    nullptr,  // default value
    size_function__SetPositionArray__positions,  // size() function pointer
    get_const_function__SetPositionArray__positions,  // get_const(index) function pointer
    get_function__SetPositionArray__positions,  // get(index) function pointer
    fetch_function__SetPositionArray__positions,  // fetch(index, &value) function pointer
    assign_function__SetPositionArray__positions,  // assign(index, value) function pointer
    resize_function__SetPositionArray__positions  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers SetPositionArray_message_members = {
  "dynamixel_sdk_custom_interfaces::msg",  // message namespace
  "SetPositionArray",  // message name
  1,  // number of fields
  sizeof(dynamixel_sdk_custom_interfaces::msg::SetPositionArray),
  SetPositionArray_message_member_array,  // message members
  SetPositionArray_init_function,  // function to initialize message memory (memory has to be allocated)
  SetPositionArray_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t SetPositionArray_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &SetPositionArray_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace dynamixel_sdk_custom_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<dynamixel_sdk_custom_interfaces::msg::SetPositionArray>()
{
  return &::dynamixel_sdk_custom_interfaces::msg::rosidl_typesupport_introspection_cpp::SetPositionArray_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, dynamixel_sdk_custom_interfaces, msg, SetPositionArray)() {
  return &::dynamixel_sdk_custom_interfaces::msg::rosidl_typesupport_introspection_cpp::SetPositionArray_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
