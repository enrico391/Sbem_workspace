#include "diffdrive_sbem/controller_response.hpp"
#include <cassert>
#include <cmath>
#include <iostream>

int main()
{
  diffdrive_sbem::ControllerResponse result;
  const std::string prefix = "1 -2 24.5 125 5000 6000 20000 11000 0 5000 9 0 0 20000 0";
  assert(diffdrive_sbem::parse_controller_response(
    prefix + " 1.25 -2.5 9.80665 0.01 -0.02 0.03 2000 42 1 0\r\n", result));
  assert(result.left == 1 && result.right == -2);
  assert(result.current == 0.125 && result.odrive_left_ms == 6);
  assert(result.imu_valid && result.imu[0] == 1.25 && result.imu[5] == 0.03);

  // Old firmware remains usable; a missing/invalid IMU never retains fresh data.
  for (const auto &text : {std::string("1 2 24 100\r\n"), prefix,
      prefix + " 1 2 3 4 5 6 100001 42 1 0",
      prefix + " 1 2 3 4 5 6 0 42 0 1",
      prefix + " 1 2 3 4 5 6 0 42 1"}) {
    assert(diffdrive_sbem::parse_controller_response(text, result));
    assert(!result.imu_valid && std::isnan(result.imu[0]));
  }
  for (const auto &text : {"", "\r\n", "ESP32 SBEM controller ready\r\n",
      "1 2 3", "1 2 3 garbage", "1 2 24 100 nan", "1 2 24 100 5oops"}) {
    assert(!diffdrive_sbem::parse_controller_response(text, result));
    assert(!result.imu_valid && std::isnan(result.imu[5]));
  }
  // Diagnostic counters are parsed as doubles, preserving all uint32 values.
  assert(diffdrive_sbem::parse_controller_response(
    prefix + " 1 2 3 4 5 6 0 4294967295 1 0", result));
  assert(result.imu_valid);
  std::cout << "controller response compatibility/freshness tests passed\n";
}
