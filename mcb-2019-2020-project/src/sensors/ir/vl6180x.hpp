/*

*/

#ifndef VL6180X_H
#define VL6180X_H

#include "ir_sensor.hpp"
#include "vl6180x_reg.hpp"

namespace aruwlib {

namespace sensors {

// Adafruit short distance IR sensor
class VL6180X: public IRSensor {
 public:
    // Initialize sensor and I2C
    virtual void init();

    // Read sensor and updates current distance
    virtual float read();

 private:
    // Write 8 bits to the given address
    void write8(uint16_t address, uint8_t data);

    // Write 16 bits to the given address
    void write16(uint16_t address, uint8_t data);

    // Read 8 bits to the given address
    uint8_t read8(uint16_t address);

    // Read 16 bits to the given address
    uint16_t read16(uint16_t address);
};

} // namespace sensors

} // namespace aruwlib

#endif // VL6180X_H