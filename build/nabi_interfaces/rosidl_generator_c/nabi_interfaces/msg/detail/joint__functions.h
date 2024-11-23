// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from nabi_interfaces:msg/Joint.idl
// generated code does not contain a copyright notice

#ifndef NABI_INTERFACES__MSG__DETAIL__JOINT__FUNCTIONS_H_
#define NABI_INTERFACES__MSG__DETAIL__JOINT__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "nabi_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "nabi_interfaces/msg/detail/joint__struct.h"

/// Initialize msg/Joint message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * nabi_interfaces__msg__Joint
 * )) before or use
 * nabi_interfaces__msg__Joint__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_nabi_interfaces
bool
nabi_interfaces__msg__Joint__init(nabi_interfaces__msg__Joint * msg);

/// Finalize msg/Joint message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_nabi_interfaces
void
nabi_interfaces__msg__Joint__fini(nabi_interfaces__msg__Joint * msg);

/// Create msg/Joint message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * nabi_interfaces__msg__Joint__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_nabi_interfaces
nabi_interfaces__msg__Joint *
nabi_interfaces__msg__Joint__create();

/// Destroy msg/Joint message.
/**
 * It calls
 * nabi_interfaces__msg__Joint__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_nabi_interfaces
void
nabi_interfaces__msg__Joint__destroy(nabi_interfaces__msg__Joint * msg);

/// Check for msg/Joint message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_nabi_interfaces
bool
nabi_interfaces__msg__Joint__are_equal(const nabi_interfaces__msg__Joint * lhs, const nabi_interfaces__msg__Joint * rhs);

/// Copy a msg/Joint message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_nabi_interfaces
bool
nabi_interfaces__msg__Joint__copy(
  const nabi_interfaces__msg__Joint * input,
  nabi_interfaces__msg__Joint * output);

/// Initialize array of msg/Joint messages.
/**
 * It allocates the memory for the number of elements and calls
 * nabi_interfaces__msg__Joint__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_nabi_interfaces
bool
nabi_interfaces__msg__Joint__Sequence__init(nabi_interfaces__msg__Joint__Sequence * array, size_t size);

/// Finalize array of msg/Joint messages.
/**
 * It calls
 * nabi_interfaces__msg__Joint__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_nabi_interfaces
void
nabi_interfaces__msg__Joint__Sequence__fini(nabi_interfaces__msg__Joint__Sequence * array);

/// Create array of msg/Joint messages.
/**
 * It allocates the memory for the array and calls
 * nabi_interfaces__msg__Joint__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_nabi_interfaces
nabi_interfaces__msg__Joint__Sequence *
nabi_interfaces__msg__Joint__Sequence__create(size_t size);

/// Destroy array of msg/Joint messages.
/**
 * It calls
 * nabi_interfaces__msg__Joint__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_nabi_interfaces
void
nabi_interfaces__msg__Joint__Sequence__destroy(nabi_interfaces__msg__Joint__Sequence * array);

/// Check for msg/Joint message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_nabi_interfaces
bool
nabi_interfaces__msg__Joint__Sequence__are_equal(const nabi_interfaces__msg__Joint__Sequence * lhs, const nabi_interfaces__msg__Joint__Sequence * rhs);

/// Copy an array of msg/Joint messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_nabi_interfaces
bool
nabi_interfaces__msg__Joint__Sequence__copy(
  const nabi_interfaces__msg__Joint__Sequence * input,
  nabi_interfaces__msg__Joint__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // NABI_INTERFACES__MSG__DETAIL__JOINT__FUNCTIONS_H_
