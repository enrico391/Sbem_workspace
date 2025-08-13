#ifndef DIFFDRIVE_ARDUINO_WHEEL_HPP
#define DIFFDRIVE_ARDUINO_WHEEL_HPP

#include <string>
#include <cmath>


class Wheel
{
    private:
    double last_enc = 0.0;
    bool first_read = true;
    
    public:

    std::string name = "";
    double enc = 0;  // Changed from float to double for precision
    double cmd = 0;
    double pos = 0;
    double vel = 0;
    double rads_per_count = 0;
    int counts_per_rev = 0;

    Wheel() = default;

    Wheel(const std::string &wheel_name, int counts_per_rev)
    {
      setup(wheel_name, counts_per_rev);
    }

    
    void setup(const std::string &wheel_name, int counts_per_revolution)
    {
      name = wheel_name;
      counts_per_rev = counts_per_revolution;
      rads_per_count = (2.0 * M_PI) / counts_per_revolution;
    }

    double calc_enc_angle()
    {
        if (first_read) {
            last_enc = enc;
            first_read = false;
            return pos; // Return current position, don't change it
        }
        
        // Calculate the difference since last reading
        double enc_diff = enc - last_enc;
        
        // Handle potential encoder overflow/underflow
        if (enc_diff > (counts_per_rev / 2.0)) {
            enc_diff -= counts_per_rev;
        } else if (enc_diff < -(counts_per_rev / 2.0)) {
            enc_diff += counts_per_rev;
        }
        
        // Update accumulated position
        pos += enc_diff * rads_per_count;
        last_enc = enc;
        
        return pos;
    }

};


#endif // DIFFDRIVE_ARDUINO_WHEEL_HPP