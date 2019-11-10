/*

*/

#ifndef ANALOGIR_H
#define ANALOGIR_H

#include "ir_sensor.hpp"
#include "analog.hpp"

namespace aruwlib {

namespace sensors {

class AnalogIR: public IRSensor {
 public:
  // Constructor to init boundaries and disance calculation values
  AnalogIR(float minDistance, float maxDistance, float m, float b, Analog::Pin pin);

  // Read sensor and updates current distance
  float read();

 private:
    // Distance calulation values for linear model y=mx+b
    float m;
    float b;

    // Analog pin
    Analog::Pin pin;
};

} // namespace sensors

} // namespace aruwlib

#endif // ANALOGIR_H