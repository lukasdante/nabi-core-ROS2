// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from nabi_interfaces:msg/Joint.idl
// generated code does not contain a copyright notice

#ifndef NABI_INTERFACES__MSG__DETAIL__JOINT__TRAITS_HPP_
#define NABI_INTERFACES__MSG__DETAIL__JOINT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "nabi_interfaces/msg/detail/joint__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace nabi_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const Joint & msg,
  std::ostream & out)
{
  out << "{";
  // member: can_id
  {
    out << "can_id: ";
    rosidl_generator_traits::value_to_yaml(msg.can_id, out);
    out << ", ";
  }

  // member: joint_angle
  {
    out << "joint_angle: ";
    rosidl_generator_traits::value_to_yaml(msg.joint_angle, out);
    out << ", ";
  }

  // member: velocity
  {
    out << "velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.velocity, out);
    out << ", ";
  }

  // member: acceleration
  {
    out << "acceleration: ";
    rosidl_generator_traits::value_to_yaml(msg.acceleration, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Joint & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: can_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "can_id: ";
    rosidl_generator_traits::value_to_yaml(msg.can_id, out);
    out << "\n";
  }

  // member: joint_angle
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "joint_angle: ";
    rosidl_generator_traits::value_to_yaml(msg.joint_angle, out);
    out << "\n";
  }

  // member: velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.velocity, out);
    out << "\n";
  }

  // member: acceleration
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "acceleration: ";
    rosidl_generator_traits::value_to_yaml(msg.acceleration, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Joint & msg, bool use_flow_style = false)
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

}  // namespace nabi_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use nabi_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const nabi_interfaces::msg::Joint & msg,
  std::ostream & out, size_t indentation = 0)
{
  nabi_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use nabi_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const nabi_interfaces::msg::Joint & msg)
{
  return nabi_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<nabi_interfaces::msg::Joint>()
{
  return "nabi_interfaces::msg::Joint";
}

template<>
inline const char * name<nabi_interfaces::msg::Joint>()
{
  return "nabi_interfaces/msg/Joint";
}

template<>
struct has_fixed_size<nabi_interfaces::msg::Joint>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<nabi_interfaces::msg::Joint>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<nabi_interfaces::msg::Joint>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // NABI_INTERFACES__MSG__DETAIL__JOINT__TRAITS_HPP_
