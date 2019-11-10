#include "analog_ir.hpp"

namespace aruwlib {

namespace sensors {
    // Constructor to init boundaries and disance calculation values
    // y = 137500x + 1125 recommended for SHARP
    AnalogIR::AnalogIR(float minDistance, float maxDistance, float m, float b): IRSensor(minDistance, maxDistance), m(m), b(b), pin(Analog::Pin::T) {}

    // Read sensor and updates current distance
    float AnalogIR::read() {
        // TODO: mV is the analog input reading in mV
        distance = 1 / ((Analog::read(pin) - b) / m);
        return getDistance();
    }
} // namespace sensors

} // namespace aruwlib