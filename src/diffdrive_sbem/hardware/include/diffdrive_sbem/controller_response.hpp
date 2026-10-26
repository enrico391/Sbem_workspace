#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace diffdrive_sbem
{
struct ControllerResponse
{
  double left = 0, right = 0, voltage = 0, current = 0;
  double odrive_right_ms = std::numeric_limits<double>::quiet_NaN();
  double odrive_left_ms = std::numeric_limits<double>::quiet_NaN();
  // Acceleration XYZ (m/s^2), then angular velocity XYZ (rad/s).
  std::array<double, 6> imu;
  bool imu_valid = false;

  ControllerResponse() { imu.fill(std::numeric_limits<double>::quiet_NaN()); }
};

inline bool parse_controller_response(const std::string &text, ControllerResponse &out)
{
  out = ControllerResponse{};
  std::istringstream stream(text);
  std::vector<double> values;
  double value;
  while (stream >> value) {
    if (!std::isfinite(value) || values.size() >= 64) return false;
    values.push_back(value);
  }
  if (!stream.eof() || values.size() < 4) return false;
  out.left = values[0];
  out.right = values[1];
  out.voltage = values[2];
  out.current = values[3] / 1000.0;
  if (values.size() >= 6) {
    out.odrive_right_ms = values[4] / 1000.0;
    out.odrive_left_ms = values[5] / 1000.0;
  }
  // Preserve the first 15 fields; accept IMU data only when its complete
  // extension explicitly marks a sample fresh. Older firmware has no extension.
  if (values.size() >= 25 && values[21] >= 0 && values[21] <= 100000 &&
      values[22] >= 0 && values[22] <= UINT32_MAX &&
      std::floor(values[22]) == values[22] && values[23] == 1) {
    for (size_t n = 0; n < out.imu.size(); ++n) out.imu[n] = values[15 + n];
    out.imu_valid = true;
  }
  return true;
}
}  // namespace diffdrive_sbem
