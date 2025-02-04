// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from dynamixel_sdk_custom_interfaces:msg/SetPositionArray.idl
// generated code does not contain a copyright notice

#ifndef DYNAMIXEL_SDK_CUSTOM_INTERFACES__MSG__DETAIL__SET_POSITION_ARRAY__STRUCT_H_
#define DYNAMIXEL_SDK_CUSTOM_INTERFACES__MSG__DETAIL__SET_POSITION_ARRAY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'positions'
#include "dynamixel_sdk_custom_interfaces/msg/detail/set_position__struct.h"

/// Struct defined in msg/SetPositionArray in the package dynamixel_sdk_custom_interfaces.
typedef struct dynamixel_sdk_custom_interfaces__msg__SetPositionArray
{
  dynamixel_sdk_custom_interfaces__msg__SetPosition__Sequence positions;
} dynamixel_sdk_custom_interfaces__msg__SetPositionArray;

// Struct for a sequence of dynamixel_sdk_custom_interfaces__msg__SetPositionArray.
typedef struct dynamixel_sdk_custom_interfaces__msg__SetPositionArray__Sequence
{
  dynamixel_sdk_custom_interfaces__msg__SetPositionArray * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dynamixel_sdk_custom_interfaces__msg__SetPositionArray__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DYNAMIXEL_SDK_CUSTOM_INTERFACES__MSG__DETAIL__SET_POSITION_ARRAY__STRUCT_H_
