#include "analog_ir.hpp"

namespace aruwlib {

namespace sensors {
    // Constructor to init boundaries and disance calculation values
    AnalogIR::AnalogIR(float minDistance, float maxDistance, float m, float b): IRSensor(minDistance, maxDistance), m(m), b(b) {}

    // Read sensor and updates current distance
    void AnalogIR::read() {}
} // namespace sensors

} // namespace aruwlib