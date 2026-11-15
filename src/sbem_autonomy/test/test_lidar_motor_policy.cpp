// Copyright 2026 Enrico Moro
// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include "sbem_autonomy/lidar_motor_policy.hpp"

using sbem_autonomy::LidarMotorPolicy;
using Status = LidarMotorPolicy::Status;

TEST(LidarMotorPolicy, StartsOnAndDebouncesChargingAndManualRemoval)
{
  LidarMotorPolicy policy(2.0, 5.0);
  EXPECT_TRUE(policy.motor_on(0.0));
  policy.update(Status::Charging, 1.0);
  EXPECT_TRUE(policy.motor_on(2.9));
  policy.update(Status::Charging, 3.0);
  EXPECT_FALSE(policy.motor_on(3.0));
  policy.update(Status::Discharging, 4.0);
  EXPECT_FALSE(policy.motor_on(5.9));
  policy.update(Status::Discharging, 6.0);
  EXPECT_TRUE(policy.motor_on(6.0));
}

TEST(LidarMotorPolicy, InterruptedChargingCannotStopMotor)
{
  LidarMotorPolicy policy(2.0, 5.0);
  policy.update(Status::Charging, 0.0);
  policy.update(Status::Discharging, 1.0);
  policy.update(Status::Charging, 2.0);
  EXPECT_TRUE(policy.motor_on(3.0));
  EXPECT_FALSE(policy.motor_on(4.0));
}

TEST(LidarMotorPolicy, UnknownAndStaleDataRestoreMotorAndResetDebounce)
{
  LidarMotorPolicy policy(2.0, 5.0);
  policy.update(Status::Charging, 0.0);
  EXPECT_FALSE(policy.motor_on(2.0));
  policy.update(Status::Unknown, 2.1);
  EXPECT_TRUE(policy.motor_on(2.1));
  policy.update(Status::Charging, 3.0);
  EXPECT_FALSE(policy.motor_on(5.0));
  EXPECT_TRUE(policy.motor_on(8.1));
  policy.update(Status::Charging, 9.0);
  EXPECT_TRUE(policy.motor_on(9.0));
  EXPECT_FALSE(policy.motor_on(11.0));
}

TEST(LidarMotorPolicy, RejectsInvalidTiming)
{
  EXPECT_THROW(LidarMotorPolicy(-1.0, 5.0), std::invalid_argument);
  EXPECT_THROW(LidarMotorPolicy(2.0, 0.0), std::invalid_argument);
}
