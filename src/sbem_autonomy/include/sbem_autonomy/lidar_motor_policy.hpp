// Copyright 2026 Enrico Moro
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cmath>
#include <stdexcept>

namespace sbem_autonomy
{
class LidarMotorPolicy
{
public:
  enum class Status {Charging, Discharging, Unknown};

  LidarMotorPolicy(double debounce, double timeout)
  : debounce_(debounce), timeout_(timeout)
  {
    if (!std::isfinite(debounce) || debounce < 0.0 ||
      !std::isfinite(timeout) || timeout <= 0.0)
    {
      throw std::invalid_argument("Invalid lidar battery timing");
    }
  }

  void update(Status status, double now)
  {
    // A gap between messages also resets debounce if no timer ran during it.
    motor_on(now);
    if (!received_ || status != status_) {
      since_ = now;
    }
    received_ = true;
    status_ = status;
    last_received_ = now;
  }

  bool motor_on(double now)
  {
    if (!received_ || now - last_received_ >= timeout_) {
      received_ = false;
      on_ = true;
    } else if (status_ == Status::Unknown) {
      on_ = true;
    } else if (now - since_ >= debounce_) {
      on_ = status_ == Status::Discharging;
    }
    return on_;
  }

private:
  double debounce_;
  double timeout_;
  double since_{0.0};
  double last_received_{0.0};
  Status status_{Status::Unknown};
  bool received_{false};
  bool on_{true};
};
}  // namespace sbem_autonomy
