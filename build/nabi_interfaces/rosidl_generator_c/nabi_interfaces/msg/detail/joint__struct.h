// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from nabi_interfaces:msg/Joint.idl
// generated code does not contain a copyright notice

#ifndef NABI_INTERFACES__MSG__DETAIL__JOINT__STRUCT_H_
#define NABI_INTERFACES__MSG__DETAIL__JOINT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Joint in the package nabi_interfaces.
typedef struct nabi_interfaces__msg__Joint
{
  uint8_t can_id;
  int16_t joint_angle;
  uint16_t velocity;
  uint8_t acceleration;
} nabi_interfaces__msg__Joint;

// Struct for a sequence of nabi_interfaces__msg__Joint.
typedef struct nabi_interfaces__msg__Joint__Sequence
{
  nabi_interfaces__msg__Joint * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} nabi_interfaces__msg__Joint__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // NABI_INTERFACES__MSG__DETAIL__JOINT__STRUCT_H_
