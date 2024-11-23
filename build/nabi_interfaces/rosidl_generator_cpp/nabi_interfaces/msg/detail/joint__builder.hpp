// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from nabi_interfaces:msg/Joint.idl
// generated code does not contain a copyright notice

#ifndef NABI_INTERFACES__MSG__DETAIL__JOINT__BUILDER_HPP_
#define NABI_INTERFACES__MSG__DETAIL__JOINT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "nabi_interfaces/msg/detail/joint__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace nabi_interfaces
{

namespace msg
{

namespace builder
{

class Init_Joint_acceleration
{
public:
  explicit Init_Joint_acceleration(::nabi_interfaces::msg::Joint & msg)
  : msg_(msg)
  {}
  ::nabi_interfaces::msg::Joint acceleration(::nabi_interfaces::msg::Joint::_acceleration_type arg)
  {
    msg_.acceleration = std::move(arg);
    return std::move(msg_);
  }

private:
  ::nabi_interfaces::msg::Joint msg_;
};

class Init_Joint_velocity
{
public:
  explicit Init_Joint_velocity(::nabi_interfaces::msg::Joint & msg)
  : msg_(msg)
  {}
  Init_Joint_acceleration velocity(::nabi_interfaces::msg::Joint::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return Init_Joint_acceleration(msg_);
  }

private:
  ::nabi_interfaces::msg::Joint msg_;
};

class Init_Joint_joint_angle
{
public:
  explicit Init_Joint_joint_angle(::nabi_interfaces::msg::Joint & msg)
  : msg_(msg)
  {}
  Init_Joint_velocity joint_angle(::nabi_interfaces::msg::Joint::_joint_angle_type arg)
  {
    msg_.joint_angle = std::move(arg);
    return Init_Joint_velocity(msg_);
  }

private:
  ::nabi_interfaces::msg::Joint msg_;
};

class Init_Joint_can_id
{
public:
  Init_Joint_can_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Joint_joint_angle can_id(::nabi_interfaces::msg::Joint::_can_id_type arg)
  {
    msg_.can_id = std::move(arg);
    return Init_Joint_joint_angle(msg_);
  }

private:
  ::nabi_interfaces::msg::Joint msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::nabi_interfaces::msg::Joint>()
{
  return nabi_interfaces::msg::builder::Init_Joint_can_id();
}

}  // namespace nabi_interfaces

#endif  // NABI_INTERFACES__MSG__DETAIL__JOINT__BUILDER_HPP_
