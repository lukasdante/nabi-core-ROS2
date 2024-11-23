// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from nabi_interfaces:msg/Joint.idl
// generated code does not contain a copyright notice

#ifndef NABI_INTERFACES__MSG__DETAIL__JOINT__STRUCT_HPP_
#define NABI_INTERFACES__MSG__DETAIL__JOINT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__nabi_interfaces__msg__Joint __attribute__((deprecated))
#else
# define DEPRECATED__nabi_interfaces__msg__Joint __declspec(deprecated)
#endif

namespace nabi_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Joint_
{
  using Type = Joint_<ContainerAllocator>;

  explicit Joint_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->can_id = 0;
      this->joint_angle = 0;
      this->velocity = 0;
      this->acceleration = 0;
    }
  }

  explicit Joint_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->can_id = 0;
      this->joint_angle = 0;
      this->velocity = 0;
      this->acceleration = 0;
    }
  }

  // field types and members
  using _can_id_type =
    uint8_t;
  _can_id_type can_id;
  using _joint_angle_type =
    int16_t;
  _joint_angle_type joint_angle;
  using _velocity_type =
    uint16_t;
  _velocity_type velocity;
  using _acceleration_type =
    uint8_t;
  _acceleration_type acceleration;

  // setters for named parameter idiom
  Type & set__can_id(
    const uint8_t & _arg)
  {
    this->can_id = _arg;
    return *this;
  }
  Type & set__joint_angle(
    const int16_t & _arg)
  {
    this->joint_angle = _arg;
    return *this;
  }
  Type & set__velocity(
    const uint16_t & _arg)
  {
    this->velocity = _arg;
    return *this;
  }
  Type & set__acceleration(
    const uint8_t & _arg)
  {
    this->acceleration = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    nabi_interfaces::msg::Joint_<ContainerAllocator> *;
  using ConstRawPtr =
    const nabi_interfaces::msg::Joint_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<nabi_interfaces::msg::Joint_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<nabi_interfaces::msg::Joint_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      nabi_interfaces::msg::Joint_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<nabi_interfaces::msg::Joint_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      nabi_interfaces::msg::Joint_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<nabi_interfaces::msg::Joint_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<nabi_interfaces::msg::Joint_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<nabi_interfaces::msg::Joint_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__nabi_interfaces__msg__Joint
    std::shared_ptr<nabi_interfaces::msg::Joint_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__nabi_interfaces__msg__Joint
    std::shared_ptr<nabi_interfaces::msg::Joint_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Joint_ & other) const
  {
    if (this->can_id != other.can_id) {
      return false;
    }
    if (this->joint_angle != other.joint_angle) {
      return false;
    }
    if (this->velocity != other.velocity) {
      return false;
    }
    if (this->acceleration != other.acceleration) {
      return false;
    }
    return true;
  }
  bool operator!=(const Joint_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Joint_

// alias to use template instance with default allocator
using Joint =
  nabi_interfaces::msg::Joint_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace nabi_interfaces

#endif  // NABI_INTERFACES__MSG__DETAIL__JOINT__STRUCT_HPP_
