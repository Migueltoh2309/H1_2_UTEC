// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from unitree_hg:msg/HandCmd.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_HG__MSG__DETAIL__HAND_CMD__BUILDER_HPP_
#define UNITREE_HG__MSG__DETAIL__HAND_CMD__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "unitree_hg/msg/detail/hand_cmd__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace unitree_hg
{

namespace msg
{

namespace builder
{

class Init_HandCmd_reserve
{
public:
  explicit Init_HandCmd_reserve(::unitree_hg::msg::HandCmd & msg)
  : msg_(msg)
  {}
  ::unitree_hg::msg::HandCmd reserve(::unitree_hg::msg::HandCmd::_reserve_type arg)
  {
    msg_.reserve = std::move(arg);
    return std::move(msg_);
  }

private:
  ::unitree_hg::msg::HandCmd msg_;
};

class Init_HandCmd_motor_cmd
{
public:
  Init_HandCmd_motor_cmd()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_HandCmd_reserve motor_cmd(::unitree_hg::msg::HandCmd::_motor_cmd_type arg)
  {
    msg_.motor_cmd = std::move(arg);
    return Init_HandCmd_reserve(msg_);
  }

private:
  ::unitree_hg::msg::HandCmd msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::unitree_hg::msg::HandCmd>()
{
  return unitree_hg::msg::builder::Init_HandCmd_motor_cmd();
}

}  // namespace unitree_hg

#endif  // UNITREE_HG__MSG__DETAIL__HAND_CMD__BUILDER_HPP_
