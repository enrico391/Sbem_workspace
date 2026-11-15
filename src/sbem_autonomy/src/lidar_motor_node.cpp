// Copyright 2026 Enrico Moro
// SPDX-License-Identifier: Apache-2.0
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_srvs/srv/empty.hpp"
#include "sbem_autonomy/lidar_motor_policy.hpp"

namespace sbem_autonomy
{
class LidarMotorNode : public rclcpp::Node
{
  using Battery = sensor_msgs::msg::BatteryState;
  using Empty = std_srvs::srv::Empty;
  using Client = rclcpp::Client<Empty>;

public:
  LidarMotorNode()
  : Node("lidar_motor"),
    policy_(timing("debounce_duration", 2.0, true), timing("battery_timeout", 5.0)),
    service_timeout_(timing("service_timeout", 3.0)),
    retry_interval_(timing("retry_interval", 3.0))
  {
    const auto period = timing("check_period", 1.0);
    const auto battery_topic = topic("battery_topic", "/battery_state");
    scan_timeout_ = timing("scan_timeout", 2.0);
    ready_at_ = seconds() + timing("startup_observation_duration", 2.0, true);
    scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
      topic("scan_topic", "/scan"), rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr message) {
        if (!message->ranges.empty()) {
          last_scan_ = seconds();
        }
      });
    start_ = create_client<Empty>(topic("start_motor_service", "/start_motor"));
    stop_ = create_client<Empty>(topic("stop_motor_service", "/stop_motor"));
    battery_ = create_subscription<Battery>(
      battery_topic, rclcpp::SensorDataQoS(), [this](Battery::ConstSharedPtr message) {
        auto status = LidarMotorPolicy::Status::Unknown;
        if (message->power_supply_status == Battery::POWER_SUPPLY_STATUS_CHARGING ||
          message->power_supply_status == Battery::POWER_SUPPLY_STATUS_FULL)
        {
          status = LidarMotorPolicy::Status::Charging;
        } else if (message->power_supply_status == Battery::POWER_SUPPLY_STATUS_DISCHARGING) {
          status = LidarMotorPolicy::Status::Discharging;
        }
        policy_.update(status, seconds());
      });
    timer_ = create_wall_timer(std::chrono::duration<double>(period), [this]() {tick();});
    RCLCPP_INFO(get_logger(), "Lidar motor battery supervision ready");
  }

private:
  static double seconds()
  {
    return std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  double timing(const std::string & name, double fallback, bool allow_zero = false)
  {
    const double value = declare_parameter(name, fallback);
    if (!std::isfinite(value) || value < 0.0 || (!allow_zero && value == 0.0)) {
      throw std::invalid_argument(name + " must be finite and positive (debounce may be zero)");
    }
    return value;
  }

  std::string topic(const std::string & name, const std::string & fallback)
  {
    auto value = declare_parameter(name, fallback);
    if (value.empty()) {
      throw std::invalid_argument(name + " must not be empty");
    }
    return value;
  }

  void tick()
  {
    const double now = seconds();
    const bool desired_on = policy_.motor_on(now);
    if (now < ready_at_) {
      return;
    }
    if (pending_) {
      if (pending_->wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
        try {
          pending_->get();
          confirmed_on_ = pending_on_;
          RCLCPP_INFO(get_logger(), "Lidar motor %s request acknowledged",
            pending_on_ ? "start" : "stop");
        } catch (const std::exception & error) {
          confirmed_on_.reset();
          retry_at_ = now + retry_interval_;
          RCLCPP_WARN(get_logger(), "Motor service failed: %s", error.what());
        }
        pending_.reset();
      } else if (now - sent_at_ >= service_timeout_) {
        pending_client_->remove_pending_request(*pending_);
        pending_.reset();
        confirmed_on_.reset();
        retry_at_ = now + retry_interval_;
        RCLCPP_WARN(get_logger(),
          "Motor service %s timed out after %.1fs (requested %s, now want %s); "
          "driver may be blocked; retrying latest desired state",
          pending_client_->get_service_name(), now - sent_at_,
          pending_on_ ? "start" : "stop", desired_on ? "start" : "stop");
      } else {
        return;
      }
    }
    if ((confirmed_on_ && *confirmed_on_ == desired_on) || now < retry_at_) {
      return;
    }
    // RPLIDAR start_motor reconfigures the scanner even when already running.
    // Observe scans before sending an unnecessary restart at supervisor startup.
    if (desired_on && last_scan_ && now - *last_scan_ < scan_timeout_ &&
      (!sent_command_ || (pending_on_ && *last_scan_ > sent_at_)))
    {
      confirmed_on_ = true;
      RCLCPP_INFO(get_logger(), "Lidar already publishing scans; skipping redundant start");
      return;
    }
    pending_client_ = desired_on ? start_ : stop_;
    if (!pending_client_->service_is_ready()) {
      retry_at_ = now + retry_interval_;
      RCLCPP_WARN(get_logger(), "Motor service %s unavailable; will retry",
        pending_client_->get_service_name());
      return;
    }
    try {
      pending_on_ = desired_on;
      sent_at_ = now;
      RCLCPP_INFO(get_logger(), "Requesting lidar motor %s via %s",
        desired_on ? "start" : "stop", pending_client_->get_service_name());
      pending_.emplace(pending_client_->async_send_request(std::make_shared<Empty::Request>()));
      sent_command_ = true;
    } catch (const std::exception & error) {
      confirmed_on_.reset();
      retry_at_ = now + retry_interval_;
      RCLCPP_WARN(get_logger(), "Cannot send motor command: %s", error.what());
    }
  }

  LidarMotorPolicy policy_;
  double service_timeout_;
  double retry_interval_;
  double sent_at_{0.0};
  double retry_at_{0.0};
  double scan_timeout_{2.0};
  double ready_at_{0.0};
  std::optional<double> last_scan_;
  bool pending_on_{true};
  bool sent_command_{false};
  std::optional<bool> confirmed_on_;
  Client::SharedPtr start_, stop_, pending_client_;
  std::optional<Client::FutureAndRequestId> pending_;
  rclcpp::Subscription<Battery>::SharedPtr battery_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace sbem_autonomy

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    rclcpp::spin(std::make_shared<sbem_autonomy::LidarMotorNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("lidar_motor"), "%s", error.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
