// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from unitree_hg:msg/PressSensorState.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_HG__MSG__DETAIL__PRESS_SENSOR_STATE__BUILDER_HPP_
#define UNITREE_HG__MSG__DETAIL__PRESS_SENSOR_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "unitree_hg/msg/detail/press_sensor_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace unitree_hg
{

namespace msg
{

namespace builder
{

class Init_PressSensorState_reserve
{
public:
  explicit Init_PressSensorState_reserve(::unitree_hg::msg::PressSensorState & msg)
  : msg_(msg)
  {}
  ::unitree_hg::msg::PressSensorState reserve(::unitree_hg::msg::PressSensorState::_reserve_type arg)
  {
    msg_.reserve = std::move(arg);
    return std::move(msg_);
  }

private:
  ::unitree_hg::msg::PressSensorState msg_;
};

class Init_PressSensorState_lost
{
public:
  explicit Init_PressSensorState_lost(::unitree_hg::msg::PressSensorState & msg)
  : msg_(msg)
  {}
  Init_PressSensorState_reserve lost(::unitree_hg::msg::PressSensorState::_lost_type arg)
  {
    msg_.lost = std::move(arg);
    return Init_PressSensorState_reserve(msg_);
  }

private:
  ::unitree_hg::msg::PressSensorState msg_;
};

class Init_PressSensorState_temperature
{
public:
  explicit Init_PressSensorState_temperature(::unitree_hg::msg::PressSensorState & msg)
  : msg_(msg)
  {}
  Init_PressSensorState_lost temperature(::unitree_hg::msg::PressSensorState::_temperature_type arg)
  {
    msg_.temperature = std::move(arg);
    return Init_PressSensorState_lost(msg_);
  }

private:
  ::unitree_hg::msg::PressSensorState msg_;
};

class Init_PressSensorState_pressure
{
public:
  Init_PressSensorState_pressure()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PressSensorState_temperature pressure(::unitree_hg::msg::PressSensorState::_pressure_type arg)
  {
    msg_.pressure = std::move(arg);
    return Init_PressSensorState_temperature(msg_);
  }

private:
  ::unitree_hg::msg::PressSensorState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::unitree_hg::msg::PressSensorState>()
{
  return unitree_hg::msg::builder::Init_PressSensorState_pressure();
}

}  // namespace unitree_hg

#endif  // UNITREE_HG__MSG__DETAIL__PRESS_SENSOR_STATE__BUILDER_HPP_
