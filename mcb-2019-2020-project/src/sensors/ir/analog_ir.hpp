/*

*/

#ifndef ANALOGIR_H
#define ANALOGIR_H

#include "ir_sensor.hpp"

namespace aruwlib {

namespace sensors {

class AnalogIR: public IRSensor {
 public:
  // Constructor to init boundaries and disance calculation values
  AnalogIR(float minDistance, float maxDistance, float m, float b);

  // Read sensor and updates current distance
  float read();

 private:
    // Distance calulation values for linear model y=mx+b
    float m;
    float b;
};

} // namespace sensors

} // namespace aruwlib

#endif // ANALOGIR_H