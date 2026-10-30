// Copyright 2026 Enrico Moro
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cmath>
#include <stdexcept>

namespace sbem_autonomy
{
// Times are monotonic seconds, independent of ROS clock jumps or paused simulation.
class BatteryPolicy
{
public:
  BatteryPolicy(double low, double high, double scale, double debounce, double timeout)
  : low_(low), high_(high), scale_(scale), debounce_(debounce), timeout_(timeout)
  {
    if (!std::isfinite(low) || !std::isfinite(high) || !std::isfinite(scale) ||
      !std::isfinite(debounce) || !std::isfinite(timeout) ||
      low < 0.0 || high > 1.0 || low >= high || scale <= 0.0 ||
      debounce < 0.0 || timeout <= 0.0)
    {
      throw std::invalid_argument("Invalid battery thresholds, scale, or timing parameters");
    }
  }

  void update(double percentage, bool charging, double now)
  {
    if (!fresh(now)) {
      low_since_ = -1.0;
    }
    valid_ = std::isfinite(percentage) && percentage >= 0.0 && percentage <= scale_;
    received_ = now;
    charging_ = valid_ && charging;
    if (!valid_) {
      low_since_ = -1.0;
      return;
    }
    const double fraction = percentage / scale_;
    if (required_ && charging_ && fraction >= high_) {
      required_ = false;
    }
    if (fraction <= low_) {
      if (low_since_ < 0.0) {
        low_since_ = now;
      }
      if (now - low_since_ >= debounce_) {
        required_ = true;
      }
    } else {
      low_since_ = -1.0;
    }
  }

  bool fresh(double now) const
  {
    return valid_ && now >= received_ && now - received_ <= timeout_;
  }
  bool required() const {return required_;}
  bool charging() const {return charging_;}

private:
  double low_, high_, scale_, debounce_, timeout_;
  double received_{0.0};
  double low_since_{-1.0};
  bool valid_{false};
  bool required_{false};
  bool charging_{false};
};
}  // namespace sbem_autonomy
