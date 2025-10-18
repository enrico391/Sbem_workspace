#ifndef DIFFDRIVE_ARDUINO_BATTERY_HPP
#define DIFFDRIVE_ARDUINO_BATTERY_HPP

#include <string>
#include <cmath>

class Battery
{
    public:

    std::string name{""};
    double voltage{0.0};
    double current{0.0};

    Battery() = default;

    Battery(const std::string &battery_name)
    {
      setup(battery_name);
    }

    void setup(const std::string &battery_name)
    {
      name = battery_name;
    }

};


#endif // DIFFDRIVE_ARDUINO_BATTERY_HPP