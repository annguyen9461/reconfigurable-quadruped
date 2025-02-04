// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from dynamixel_sdk_custom_interfaces:msg/SetPositionArray.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "dynamixel_sdk_custom_interfaces/msg/detail/set_position_array__rosidl_typesupport_introspection_c.h"
#include "dynamixel_sdk_custom_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "dynamixel_sdk_custom_interfaces/msg/detail/set_position_array__functions.h"
#include "dynamixel_sdk_custom_interfaces/msg/detail/set_position_array__struct.h"


// Include directives for member types
// Member `positions`
#include "dynamixel_sdk_custom_interfaces/msg/set_position.h"
// Member `positions`
#include "dynamixel_sdk_custom_interfaces/msg/detail/set_position__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void dynamixel_sdk_custom_interfaces__msg__SetPositionArray__rosidl_typesupport_introspection_c__SetPositionArray_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  dynamixel_sdk_custom_interfaces__msg__SetPositionArray__init(message_memory);
}

void dynamixel_sdk_custom_interfaces__msg__SetPositionArray__rosidl_typesupport_introspection_c__SetPositionArray_fini_function(void * message_memory)
{
  dynamixel_sdk_custom_interfaces__msg__SetPositionArray__fini(message_memory);
}

size_t dynamixel_sdk_custom_interfaces__msg__SetPositionArray__rosidl_typesupport_introspection_c__size_function__SetPositionArray__positions(
  const void * untyped_member)
{
  const dynamixel_sdk_custom_interfaces__msg__SetPosition__Sequence * member =
    (const dynamixel_sdk_custom_interfaces__msg__SetPosition__Sequence *)(untyped_member);
  return member->size;
}

const void * dynamixel_sdk_custom_interfaces__msg__SetPositionArray__rosidl_typesupport_introspection_c__get_const_function__SetPositionArray__positions(
  const void * untyped_member, size_t index)
{
  const dynamixel_sdk_custom_interfaces__msg__SetPosition__Sequence * member =
    (const dynamixel_sdk_custom_interfaces__msg__SetPosition__Sequence *)(untyped_member);
  return &member->data[index];
}

void * dynamixel_sdk_custom_interfaces__msg__SetPositionArray__rosidl_typesupport_introspection_c__get_function__SetPositionArray__positions(
  void * untyped_member, size_t index)
{
  dynamixel_sdk_custom_interfaces__msg__SetPosition__Sequence * member =
    (dynamixel_sdk_custom_interfaces__msg__SetPosition__Sequence *)(untyped_member);
  return &member->data[index];
}

void dynamixel_sdk_custom_interfaces__msg__SetPositionArray__rosidl_typesupport_introspection_c__fetch_function__SetPositionArray__positions(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const dynamixel_sdk_custom_interfaces__msg__SetPosition * item =
    ((const dynamixel_sdk_custom_interfaces__msg__SetPosition *)
    dynamixel_sdk_custom_interfaces__msg__SetPositionArray__rosidl_typesupport_introspection_c__get_const_function__SetPositionArray__positions(untyped_member, index));
  dynamixel_sdk_custom_interfaces__msg__SetPosition * value =
    (dynamixel_sdk_custom_interfaces__msg__SetPosition *)(untyped_value);
  *value = *item;
}

void dynamixel_sdk_custom_interfaces__msg__SetPositionArray__rosidl_typesupport_introspection_c__assign_function__SetPositionArray__positions(
  void * untyped_member, size_t index, const void * untyped_value)
{
  dynamixel_sdk_custom_interfaces__msg__SetPosition * item =
    ((dynamixel_sdk_custom_interfaces__msg__SetPosition *)
    dynamixel_sdk_custom_interfaces__msg__SetPositionArray__rosidl_typesupport_introspection_c__get_function__SetPositionArray__positions(untyped_member, index));
  const dynamixel_sdk_custom_interfaces__msg__SetPosition * value =
    (const dynamixel_sdk_custom_interfaces__msg__SetPosition *)(untyped_value);
  *item = *value;
}

bool dynamixel_sdk_custom_interfaces__msg__SetPositionArray__rosidl_typesupport_introspection_c__resize_function__SetPositionArray__positions(
  void * untyped_member, size_t size)
{
  dynamixel_sdk_custom_interfaces__msg__SetPosition__Sequence * member =
    (dynamixel_sdk_custom_interfaces__msg__SetPosition__Sequence *)(untyped_member);
  dynamixel_sdk_custom_interfaces__msg__SetPosition__Sequence__fini(member);
  return dynamixel_sdk_custom_interfaces__msg__SetPosition__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember dynamixel_sdk_custom_interfaces__msg__SetPositionArray__rosidl_typesupport_introspection_c__SetPositionArray_message_member_array[1] = {
  {
    "positions",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dynamixel_sdk_custom_interfaces__msg__SetPositionArray, positions),  // bytes offset in struct
    NULL,  // default value
    dynamixel_sdk_custom_interfaces__msg__SetPositionArray__rosidl_typesupport_introspection_c__size_function__SetPositionArray__positions,  // size() function pointer
    dynamixel_sdk_custom_interfaces__msg__SetPositionArray__rosidl_typesupport_introspection_c__get_const_function__SetPositionArray__positions,  // get_const(index) function pointer
    dynamixel_sdk_custom_interfaces__msg__SetPositionArray__rosidl_typesupport_introspection_c__get_function__SetPositionArray__positions,  // get(index) function pointer
    dynamixel_sdk_custom_interfaces__msg__SetPositionArray__rosidl_typesupport_introspection_c__fetch_function__SetPositionArray__positions,  // fetch(index, &value) function pointer
    dynamixel_sdk_custom_interfaces__msg__SetPositionArray__rosidl_typesupport_introspection_c__assign_function__SetPositionArray__positions,  // assign(index, value) function pointer
    dynamixel_sdk_custom_interfaces__msg__SetPositionArray__rosidl_typesupport_introspection_c__resize_function__SetPositionArray__positions  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers dynamixel_sdk_custom_interfaces__msg__SetPositionArray__rosidl_typesupport_introspection_c__SetPositionArray_message_members = {
  "dynamixel_sdk_custom_interfaces__msg",  // message namespace
  "SetPositionArray",  // message name
  1,  // number of fields
  sizeof(dynamixel_sdk_custom_interfaces__msg__SetPositionArray),
  dynamixel_sdk_custom_interfaces__msg__SetPositionArray__rosidl_typesupport_introspection_c__SetPositionArray_message_member_array,  // message members
  dynamixel_sdk_custom_interfaces__msg__SetPositionArray__rosidl_typesupport_introspection_c__SetPositionArray_init_function,  // function to initialize message memory (memory has to be allocated)
  dynamixel_sdk_custom_interfaces__msg__SetPositionArray__rosidl_typesupport_introspection_c__SetPositionArray_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t dynamixel_sdk_custom_interfaces__msg__SetPositionArray__rosidl_typesupport_introspection_c__SetPositionArray_message_type_support_handle = {
  0,
  &dynamixel_sdk_custom_interfaces__msg__SetPositionArray__rosidl_typesupport_introspection_c__SetPositionArray_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_dynamixel_sdk_custom_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dynamixel_sdk_custom_interfaces, msg, SetPositionArray)() {
  dynamixel_sdk_custom_interfaces__msg__SetPositionArray__rosidl_typesupport_introspection_c__SetPositionArray_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dynamixel_sdk_custom_interfaces, msg, SetPosition)();
  if (!dynamixel_sdk_custom_interfaces__msg__SetPositionArray__rosidl_typesupport_introspection_c__SetPositionArray_message_type_support_handle.typesupport_identifier) {
    dynamixel_sdk_custom_interfaces__msg__SetPositionArray__rosidl_typesupport_introspection_c__SetPositionArray_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &dynamixel_sdk_custom_interfaces__msg__SetPositionArray__rosidl_typesupport_introspection_c__SetPositionArray_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
