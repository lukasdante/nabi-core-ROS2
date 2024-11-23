// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from nabi_interfaces:msg/Joint.idl
// generated code does not contain a copyright notice
#include "nabi_interfaces/msg/detail/joint__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
nabi_interfaces__msg__Joint__init(nabi_interfaces__msg__Joint * msg)
{
  if (!msg) {
    return false;
  }
  // can_id
  // joint_angle
  // velocity
  // acceleration
  return true;
}

void
nabi_interfaces__msg__Joint__fini(nabi_interfaces__msg__Joint * msg)
{
  if (!msg) {
    return;
  }
  // can_id
  // joint_angle
  // velocity
  // acceleration
}

bool
nabi_interfaces__msg__Joint__are_equal(const nabi_interfaces__msg__Joint * lhs, const nabi_interfaces__msg__Joint * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // can_id
  if (lhs->can_id != rhs->can_id) {
    return false;
  }
  // joint_angle
  if (lhs->joint_angle != rhs->joint_angle) {
    return false;
  }
  // velocity
  if (lhs->velocity != rhs->velocity) {
    return false;
  }
  // acceleration
  if (lhs->acceleration != rhs->acceleration) {
    return false;
  }
  return true;
}

bool
nabi_interfaces__msg__Joint__copy(
  const nabi_interfaces__msg__Joint * input,
  nabi_interfaces__msg__Joint * output)
{
  if (!input || !output) {
    return false;
  }
  // can_id
  output->can_id = input->can_id;
  // joint_angle
  output->joint_angle = input->joint_angle;
  // velocity
  output->velocity = input->velocity;
  // acceleration
  output->acceleration = input->acceleration;
  return true;
}

nabi_interfaces__msg__Joint *
nabi_interfaces__msg__Joint__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  nabi_interfaces__msg__Joint * msg = (nabi_interfaces__msg__Joint *)allocator.allocate(sizeof(nabi_interfaces__msg__Joint), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(nabi_interfaces__msg__Joint));
  bool success = nabi_interfaces__msg__Joint__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
nabi_interfaces__msg__Joint__destroy(nabi_interfaces__msg__Joint * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    nabi_interfaces__msg__Joint__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
nabi_interfaces__msg__Joint__Sequence__init(nabi_interfaces__msg__Joint__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  nabi_interfaces__msg__Joint * data = NULL;

  if (size) {
    data = (nabi_interfaces__msg__Joint *)allocator.zero_allocate(size, sizeof(nabi_interfaces__msg__Joint), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = nabi_interfaces__msg__Joint__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        nabi_interfaces__msg__Joint__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
nabi_interfaces__msg__Joint__Sequence__fini(nabi_interfaces__msg__Joint__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      nabi_interfaces__msg__Joint__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

nabi_interfaces__msg__Joint__Sequence *
nabi_interfaces__msg__Joint__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  nabi_interfaces__msg__Joint__Sequence * array = (nabi_interfaces__msg__Joint__Sequence *)allocator.allocate(sizeof(nabi_interfaces__msg__Joint__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = nabi_interfaces__msg__Joint__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
nabi_interfaces__msg__Joint__Sequence__destroy(nabi_interfaces__msg__Joint__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    nabi_interfaces__msg__Joint__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
nabi_interfaces__msg__Joint__Sequence__are_equal(const nabi_interfaces__msg__Joint__Sequence * lhs, const nabi_interfaces__msg__Joint__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!nabi_interfaces__msg__Joint__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
nabi_interfaces__msg__Joint__Sequence__copy(
  const nabi_interfaces__msg__Joint__Sequence * input,
  nabi_interfaces__msg__Joint__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(nabi_interfaces__msg__Joint);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    nabi_interfaces__msg__Joint * data =
      (nabi_interfaces__msg__Joint *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!nabi_interfaces__msg__Joint__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          nabi_interfaces__msg__Joint__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!nabi_interfaces__msg__Joint__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
