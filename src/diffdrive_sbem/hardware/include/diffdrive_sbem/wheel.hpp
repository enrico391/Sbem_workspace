#ifndef DIFFDRIVE_ARDUINO_WHEEL_HPP
#define DIFFDRIVE_ARDUINO_WHEEL_HPP

#include <string>
#include <cmath>


class Wheel
{
    public:

    std::string name = "";
    double cmd = 0;
    double pos = 0;
    double vel = 0;
    float turns = 0;
    int reverse = 1; // 1 for normal, -1 for reverse

    Wheel() = default;

    Wheel(const std::string &wheel_name, int rev)
    {
      setup(wheel_name, rev);
    }

    void setup(const std::string &wheel_name, int rev)
    {
      reverse = rev;
      name = wheel_name;
    }

    double update_from_turns()
    {
      return turns * 2 * M_PI * reverse;
    }

};


#endif // DIFFDRIVE_ARDUINO_WHEEL_HPP