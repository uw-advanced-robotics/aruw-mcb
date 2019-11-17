/*
Basic Analog IR Sensor via analog input
The distance conversion can be tweaked depending on the sensor
*/

#ifndef ANALOGIR_H
#define ANALOGIR_H

#include "ir_sensor.hpp"
#include "src/communication/gpio/analog.hpp"

namespace aruwlib {

namespace sensors {

class AnalogIR: public IRSensor {
 public:
   // Constructor to init boundaries
   // By default set m/b/offset values to SHARP IR and analog pin to S
   AnalogIR(float minDistance, float maxDistance);

   // Constructor to init boundaries and analog pin
   // By default set m/b/offset values to SHARP IR
   AnalogIR(float minDistance, float maxDistance, gpio::Analog::Pin pin);

   // Constructor to init boundaries, disance calculation values, and analog pin
   AnalogIR(float minDistance, float maxDistance, float m, float b, float offset, gpio::Analog::Pin pin);

   // Initialize sensor and ADC
   void init();

   // Read sensor and updates current distance
   float read();

 private:
   // Distance calculation values for SHARP 0A41SK F IR Sensor
   // Subject to change
   #define SHARPIR_m 0.072
   #define SHARPIR_b -0.008
   #define SHARPIR_offset -0.42

   // Distance calulation values for linear model y=mx+b
   float m_m;
   float m_b;

   // Offset value of inverse
   float m_offset;

   // Analog pin
   gpio::Analog::Pin m_pin;
};

} // namespace sensors

} // namespace aruwlib

#endif // ANALOGIR_H