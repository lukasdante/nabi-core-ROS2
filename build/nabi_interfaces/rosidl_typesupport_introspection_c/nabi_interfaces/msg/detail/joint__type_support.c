// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from nabi_interfaces:msg/Joint.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "nabi_interfaces/msg/detail/joint__rosidl_typesupport_introspection_c.h"
#include "nabi_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "nabi_interfaces/msg/detail/joint__functions.h"
#include "nabi_interfaces/msg/detail/joint__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void nabi_interfaces__msg__Joint__rosidl_typesupport_introspection_c__Joint_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  nabi_interfaces__msg__Joint__init(message_memory);
}

void nabi_interfaces__msg__Joint__rosidl_typesupport_introspection_c__Joint_fini_function(void * message_memory)
{
  nabi_interfaces__msg__Joint__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember nabi_interfaces__msg__Joint__rosidl_typesupport_introspection_c__Joint_message_member_array[4] = {
  {
    "can_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(nabi_interfaces__msg__Joint, can_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "joint_angle",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(nabi_interfaces__msg__Joint, joint_angle),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "velocity",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(nabi_interfaces__msg__Joint, velocity),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "acceleration",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(nabi_interfaces__msg__Joint, acceleration),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers nabi_interfaces__msg__Joint__rosidl_typesupport_introspection_c__Joint_message_members = {
  "nabi_interfaces__msg",  // message namespace
  "Joint",  // message name
  4,  // number of fields
  sizeof(nabi_interfaces__msg__Joint),
  nabi_interfaces__msg__Joint__rosidl_typesupport_introspection_c__Joint_message_member_array,  // message members
  nabi_interfaces__msg__Joint__rosidl_typesupport_introspection_c__Joint_init_function,  // function to initialize message memory (memory has to be allocated)
  nabi_interfaces__msg__Joint__rosidl_typesupport_introspection_c__Joint_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t nabi_interfaces__msg__Joint__rosidl_typesupport_introspection_c__Joint_message_type_support_handle = {
  0,
  &nabi_interfaces__msg__Joint__rosidl_typesupport_introspection_c__Joint_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_nabi_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, nabi_interfaces, msg, Joint)() {
  if (!nabi_interfaces__msg__Joint__rosidl_typesupport_introspection_c__Joint_message_type_support_handle.typesupport_identifier) {
    nabi_interfaces__msg__Joint__rosidl_typesupport_introspection_c__Joint_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &nabi_interfaces__msg__Joint__rosidl_typesupport_introspection_c__Joint_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
