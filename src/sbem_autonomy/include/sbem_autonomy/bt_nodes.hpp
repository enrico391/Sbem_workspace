// Copyright 2026 Enrico Moro
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <array>
#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <vector>

#include "action_msgs/msg/goal_status_array.hpp"
#include "action_msgs/srv/cancel_goal.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "sbem_autonomy/battery_policy.hpp"

namespace sbem_autonomy
{
inline double steady_seconds()
{
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

class CancelNavigation : public BT::StatefulActionNode
{
public:
  CancelNavigation(const std::string & name, const BT::NodeConfiguration & config);
  static BT::PortsList providedPorts() {return {};}
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  using Cancel = action_msgs::srv::CancelGoal;
  using StatusArray = action_msgs::msg::GoalStatusArray;
  using GoalId = std::array<uint8_t, 16>;
  struct Endpoint
  {
    rclcpp::Client<Cancel>::SharedPtr client;
    rclcpp::Subscription<StatusArray>::SharedPtr subscription;
    StatusArray latest_status;
    rclcpp::Client<Cancel>::SharedFuture future;
    int64_t request_id{0};
    bool sent{false};
    bool acknowledged{false};
    std::vector<GoalId> canceling;
  };
  rclcpp::Node::SharedPtr node_;
  std::array<Endpoint, 2> endpoints_;
  double started_{0.0};
  double timeout_;
};

class WaitForRecharge : public BT::StatefulActionNode
{
public:
  WaitForRecharge(const std::string & name, const BT::NodeConfiguration & config);
  static BT::PortsList providedPorts() {return {};}
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override {}

private:
  std::shared_ptr<BatteryPolicy> battery_;
  double started_{0.0};
  double last_charging_{0.0};
  double timeout_;
  double loss_timeout_;
};
}  // namespace sbem_autonomy
