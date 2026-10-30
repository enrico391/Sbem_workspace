// Copyright 2026 Enrico Moro
// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include <limits>
#include "sbem_autonomy/battery_policy.hpp"

using sbem_autonomy::BatteryPolicy;

TEST(BatteryPolicy, DebouncesAndLatchesUntilCharged)
{
  BatteryPolicy policy(0.2, 0.8, 100.0, 5.0, 3.0);
  policy.update(19.0, false, 0.0);
  policy.update(19.0, false, 2.0);
  EXPECT_FALSE(policy.required());
  policy.update(30.0, false, 3.0);  // A recovered reading resets debounce.
  policy.update(19.0, false, 4.0);
  policy.update(19.0, false, 6.0);
  policy.update(19.0, false, 8.0);
  EXPECT_FALSE(policy.required());
  policy.update(19.0, false, 9.0);
  EXPECT_TRUE(policy.required());
  policy.update(60.0, true, 10.0);
  EXPECT_TRUE(policy.required());
  policy.update(80.0, false, 11.0);
  EXPECT_TRUE(policy.required());  // Voltage recovery alone cannot finish charging.
  policy.update(80.0, true, 12.0);
  EXPECT_FALSE(policy.required());
}

TEST(BatteryPolicy, RejectsMissingInvalidAndStaleSamples)
{
  BatteryPolicy policy(0.2, 0.8, 1.0, 2.0, 3.0);
  EXPECT_FALSE(policy.fresh(0.0));
  policy.update(0.1, false, 0.0);
  EXPECT_TRUE(policy.fresh(1.0));
  EXPECT_FALSE(policy.fresh(4.0));
  policy.update(0.1, false, 5.0);  // Missing interval must restart debounce.
  EXPECT_FALSE(policy.required());
  policy.update(std::numeric_limits<double>::quiet_NaN(), false, 6.0);
  EXPECT_FALSE(policy.fresh(6.0));
  policy.update(1.1, false, 7.0);
  EXPECT_FALSE(policy.fresh(7.0));
  policy.update(-0.1, false, 8.0);
  EXPECT_FALSE(policy.fresh(8.0));
  policy.update(0.1, false, 9.0);
  policy.update(0.1, false, 11.0);
  EXPECT_TRUE(policy.required());
  EXPECT_FALSE(policy.fresh(15.0));
  EXPECT_TRUE(policy.required());  // Staleness must not clear the recharge latch.
}

TEST(BatteryPolicy, SupportsNormalizedMessagesAndRejectsBadConfiguration)
{
  BatteryPolicy policy(0.2, 0.8, 1.0, 0.0, 3.0);
  policy.update(0.2, false, 0.0);
  EXPECT_TRUE(policy.required());
  EXPECT_THROW(BatteryPolicy(0.8, 0.2, 1.0, 0.0, 3.0), std::invalid_argument);
  EXPECT_THROW(BatteryPolicy(0.2, 0.8, 0.0, 0.0, 3.0), std::invalid_argument);
  EXPECT_THROW(BatteryPolicy(0.2, 0.8, 1.0, -1.0, 3.0), std::invalid_argument);
}
