/*

*/

#ifndef SEEEDIR_H
#define SEEEDIR_H

#include "ir_sensor.hpp"

namespace aruwlib {

namespace sensors {

class SeeedIR: public IRSensor {
 public:
    // Initialize sensor and UART
    void init();

    // Read sensor and update current distance
    virtual float read();
};

} // namespace sensors

} // namespace aruwlib

#endif // SEEED_H