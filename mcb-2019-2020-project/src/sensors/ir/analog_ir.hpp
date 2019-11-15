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
    AnalogIR(float _minDistance, float _maxDistance, float _m, float _b, gpio::Analog::Pin _pin);

    // Initialize sensor and ADC
    virtual void init();

    // Read sensor and updates current distance
    float read();

 private:
    // Distance calulation values for linear model y=mx+b
    float m;
    float b;

    // Analog pin
    gpio::Analog::Pin pin;
};

} // namespace sensors

} // namespace aruwlib

#endif // ANALOGIR_H