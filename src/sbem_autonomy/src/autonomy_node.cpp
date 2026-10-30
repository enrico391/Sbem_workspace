// Copyright 2026 Enrico Moro
// SPDX-License-Identifier: Apache-2.0
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"
#include "sensor_msgs/msg/battery_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "sbem_autonomy/bt_nodes.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("sbem_autonomy");
  try {
    const auto positive = [&node](const std::string & name, double value) {
        const double result = node->declare_parameter(name, value);
        if (!std::isfinite(result) || result <= 0.0) {
          throw std::invalid_argument(name + " must be finite and positive");
        }
        return result;
      };
    const auto low = node->declare_parameter("low_battery_threshold", 0.20);
    const auto high = node->declare_parameter("recharge_complete_threshold", 0.80);
    const auto debounce = node->declare_parameter("low_battery_duration", 5.0);
    auto battery = std::make_shared<sbem_autonomy::BatteryPolicy>(
      low, high, 1.0, debounce, positive("battery_timeout", 5.0));
    bool enabled = node->declare_parameter("enabled", true);
    const auto tick_ms = node->declare_parameter("tick_period_ms", 100);
    const auto max_attempts = node->declare_parameter("max_attempts", 3);
    if (tick_ms <= 0 || max_attempts <= 0) {
      throw std::invalid_argument("tick_period_ms and max_attempts must be positive");
    }
    const double retry_delay = positive("retry_delay", 30.0);
    const auto docking_timeout_ms = static_cast<unsigned>(
      positive("docking_timeout", 240.0) * 1000.0);
    const auto dock_id = node->declare_parameter<std::string>("dock_id", "charging_dock");
    if (dock_id.empty()) {
      throw std::invalid_argument("dock_id must not be empty");
    }
    const auto battery_topic = node->declare_parameter<std::string>("battery_topic", "/battery_state");
    const auto docking_action = node->declare_parameter<std::string>("docking_action", "/dock_robot");
    const auto navigation_actions = node->declare_parameter<std::vector<std::string>>(
      "navigation_actions", {"/navigate_to_pose", "/navigate_through_poses"});
    const auto tree_file = node->declare_parameter<std::string>(
      "bt_xml", ament_index_cpp::get_package_share_directory("sbem_autonomy") +
      "/behavior_trees/auto_charge.xml");

    auto blackboard = BT::Blackboard::create();
    blackboard->set("node", node);
    blackboard->set("battery", battery);
    blackboard->set("dock_id", dock_id);
    blackboard->set("docking_action", docking_action);
    blackboard->set("docking_timeout_ms", docking_timeout_ms);
    blackboard->set("navigation_actions", navigation_actions);
    blackboard->set("cancel_timeout", positive("cancel_timeout", 10.0));
    blackboard->set("charge_timeout", positive("charge_timeout", 21600.0));
    blackboard->set("charging_loss_timeout", positive("charging_loss_timeout", 30.0));
    blackboard->set("bt_loop_duration", std::chrono::milliseconds(tick_ms));
    blackboard->set("server_timeout", std::chrono::milliseconds(2000));
    blackboard->set("wait_for_service_timeout", std::chrono::milliseconds(30000));

    auto subscription = node->create_subscription<sensor_msgs::msg::BatteryState>(
      battery_topic, rclcpp::SensorDataQoS(),
      [battery](sensor_msgs::msg::BatteryState::ConstSharedPtr message) {
        using Battery = sensor_msgs::msg::BatteryState;
        const bool charging = message->power_supply_status == Battery::POWER_SUPPLY_STATUS_CHARGING ||
          message->power_supply_status == Battery::POWER_SUPPLY_STATUS_FULL;
        battery->update(message->percentage, charging, sbem_autonomy::steady_seconds());
      });
    auto state_publisher = node->create_publisher<std_msgs::msg::String>(
      "~/state", rclcpp::QoS(1).transient_local());
    auto required_publisher = node->create_publisher<std_msgs::msg::Bool>(
      "~/charging_required", rclcpp::QoS(1).transient_local());

    BT::BehaviorTreeFactory factory;
    BT::SharedLibrary library;
    factory.registerFromPlugin(library.getOSName("opennav_dock_action_bt_node"));
    factory.registerNodeType<sbem_autonomy::CancelNavigation>("CancelNavigation");
    factory.registerNodeType<sbem_autonomy::WaitForRecharge>("WaitForRecharge");
    factory.registerSimpleCondition("BatteryFresh", [battery](BT::TreeNode &) {
        return battery->fresh(sbem_autonomy::steady_seconds()) ?
               BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
      });
    factory.registerSimpleCondition("RechargeRequired", [battery](BT::TreeNode &) {
        return battery->required() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
      });
    factory.registerSimpleCondition("IsCharging", [battery](BT::TreeNode &) {
        return battery->charging() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
      });
    auto tree = factory.createTreeFromFile(tree_file, blackboard);

    int64_t attempts = 0;
    double retry_at = 0.0;
    auto enable_service = node->create_service<std_srvs::srv::SetBool>(
      "~/set_enabled", [&enabled, &attempts, &retry_at, &tree](
        std_srvs::srv::SetBool::Request::SharedPtr request,
        std_srvs::srv::SetBool::Response::SharedPtr response)
      {
        tree.haltTree();
        enabled = request->data;
        attempts = 0;
        retry_at = 0.0;
        response->success = true;
        response->message = enabled ? "Monitoring enabled; retry budget reset" : "Autonomy disabled";
      });

    std::string previous_state;
    bool charged = false;
    bool was_required = false;
    rclcpp::WallRate rate(1000.0 / tick_ms);
    RCLCPP_INFO(node->get_logger(), "Auto-charge ready: dock=%s, thresholds=%.0f%% / %.0f%%",
      dock_id.c_str(), low * 100.0, high * 100.0);
    while (rclcpp::ok()) {
      rclcpp::spin_some(node);
      const auto now = sbem_autonomy::steady_seconds();
      const bool required = battery->required();
      if (was_required && !required) {
        charged = true;
      }
      if (required) {
        charged = false;
      }
      was_required = required;
      std::string state;
      if (!enabled) {
        tree.haltTree();
        state = "disabled";
      } else if (!battery->fresh(now)) {
        tree.haltTree();
        state = "waiting_for_battery";
      } else if (attempts >= max_attempts) {
        state = "failed";
      } else if (now < retry_at) {
        state = "retry_wait";
      } else {
        const auto status = tree.tickRoot();
        if (status == BT::NodeStatus::FAILURE) {
          tree.haltTree();
          ++attempts;
          retry_at = now + retry_delay;
          state = attempts >= max_attempts ? "failed" : "retry_wait";
          RCLCPP_ERROR(node->get_logger(), "Recharge attempt %ld/%ld failed",
            static_cast<long>(attempts), static_cast<long>(max_attempts));
        } else {
          if (status == BT::NodeStatus::SUCCESS) {
            attempts = 0;
          }
          state = required ? "recharging" : (charged ? "charged" : "idle");
        }
      }
      if (state != previous_state) {
        RCLCPP_INFO(node->get_logger(), "State: %s", state.c_str());
        previous_state = state;
      }
      std_msgs::msg::String state_message;
      state_message.data = state;
      state_publisher->publish(state_message);
      std_msgs::msg::Bool required_message;
      required_message.data = required;
      required_publisher->publish(required_message);
      rate.sleep();
    }
    tree.haltTree();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(node->get_logger(), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
