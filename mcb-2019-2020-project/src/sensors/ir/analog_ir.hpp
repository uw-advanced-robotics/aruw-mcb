/*

*/

#ifndef ANALOGIR_H
#define ANALOGIR_H

#include "ir_sensor.hpp"
#include "src/communication/gpio/analog.hpp"

namespace aruwlib {

namespace sensors {

class AnalogIR: public IRSensor {
 public:
    // Constructor to init boundaries and disance calculation values
    AnalogIR(float minDistance, float maxDistance, float m, float b, gpio::Analog::Pin pin);

    // Initialize sensor and ADC
    void init();

    // Read sensor and updates current distance
    float read();

 private:
    // Distance calulation values for linear model y=mx+b
    float m_m;
    float m_b;

    // Analog pin
    gpio::Analog::Pin m_pin;
};

} // namespace sensors

} // namespace aruwlib

#endif // ANALOGIR_H