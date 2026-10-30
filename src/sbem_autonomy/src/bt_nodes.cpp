// Copyright 2026 Enrico Moro
// SPDX-License-Identifier: Apache-2.0
#include "sbem_autonomy/bt_nodes.hpp"

#include <algorithm>
#include <utility>

namespace sbem_autonomy
{
CancelNavigation::CancelNavigation(const std::string & name, const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config),
  node_(config.blackboard->get<rclcpp::Node::SharedPtr>("node")),
  timeout_(config.blackboard->get<double>("cancel_timeout"))
{
  const auto names = config.blackboard->get<std::vector<std::string>>("navigation_actions");
  if (names.size() != endpoints_.size()) {
    throw std::invalid_argument("navigation_actions must contain two action names");
  }
  for (size_t i = 0; i < endpoints_.size(); ++i) {
    auto & endpoint = endpoints_[i];
    endpoint.client = node_->create_client<Cancel>(names[i] + "/_action/cancel_goal");
    endpoint.subscription = node_->create_subscription<StatusArray>(
      names[i] + "/_action/status", rclcpp::QoS(1).reliable().transient_local(),
      [this, i](StatusArray::ConstSharedPtr msg) {endpoints_[i].latest_status = *msg;});
  }
}

BT::NodeStatus CancelNavigation::onStart()
{
  onHalted();
  started_ = steady_seconds();
  RCLCPP_INFO(node_->get_logger(), "Stopping navigation before docking");
  return onRunning();
}

BT::NodeStatus CancelNavigation::onRunning()
{
  if (steady_seconds() - started_ > timeout_) {
    RCLCPP_ERROR(node_->get_logger(), "Navigation did not stop before cancellation timeout");
    onHalted();
    return BT::NodeStatus::FAILURE;
  }
  bool stopped = true;
  for (auto & endpoint : endpoints_) {
    if (!endpoint.sent) {
      if (!endpoint.client->service_is_ready()) {
        stopped = false;
        continue;
      }
      // Zero UUID and timestamp explicitly request cancellation of all current goals.
      auto request = endpoint.client->async_send_request(std::make_shared<Cancel::Request>());
      endpoint.future = request.future.share();
      endpoint.request_id = request.request_id;
      endpoint.sent = true;
    }
    if (!endpoint.acknowledged) {
      if (endpoint.future.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
        stopped = false;
        continue;
      }
      const auto response = endpoint.future.get();
      if (response->return_code != Cancel::Response::ERROR_NONE) {
        RCLCPP_ERROR(node_->get_logger(), "Navigation cancellation rejected");
        onHalted();
        return BT::NodeStatus::FAILURE;
      }
      for (const auto & goal : response->goals_canceling) {
        endpoint.canceling.push_back(goal.goal_id.uuid);
      }
      endpoint.acknowledged = true;
    }
    // An acknowledgement only means CANCELING. Require a terminal status before moving.
    for (const auto & id : endpoint.canceling) {
      const auto match = std::find_if(
        endpoint.latest_status.status_list.begin(), endpoint.latest_status.status_list.end(),
        [&id](const auto & status) {return status.goal_info.goal_id.uuid == id;});
      if (match == endpoint.latest_status.status_list.end() ||
        match->status < action_msgs::msg::GoalStatus::STATUS_SUCCEEDED)
      {
        stopped = false;
      }
    }
  }
  return stopped ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
}

void CancelNavigation::onHalted()
{
  for (auto & endpoint : endpoints_) {
    if (endpoint.sent && !endpoint.acknowledged) {
      endpoint.client->remove_pending_request(endpoint.request_id);
    }
    endpoint.sent = false;
    endpoint.acknowledged = false;
    endpoint.canceling.clear();
  }
}

WaitForRecharge::WaitForRecharge(const std::string & name, const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config),
  battery_(config.blackboard->get<std::shared_ptr<BatteryPolicy>>("battery")),
  timeout_(config.blackboard->get<double>("charge_timeout")),
  loss_timeout_(config.blackboard->get<double>("charging_loss_timeout")) {}

BT::NodeStatus WaitForRecharge::onStart()
{
  started_ = last_charging_ = steady_seconds();
  return onRunning();
}

BT::NodeStatus WaitForRecharge::onRunning()
{
  const auto now = steady_seconds();
  if (!battery_->fresh(now)) {
    return BT::NodeStatus::FAILURE;
  }
  if (!battery_->required()) {
    return BT::NodeStatus::SUCCESS;
  }
  if (battery_->charging()) {
    last_charging_ = now;
  }
  if (now - started_ > timeout_ || now - last_charging_ > loss_timeout_) {
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}
}  // namespace sbem_autonomy
