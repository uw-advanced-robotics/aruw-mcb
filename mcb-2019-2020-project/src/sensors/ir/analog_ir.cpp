#include "analog_ir.hpp"

namespace aruwlib {

namespace sensors {
    // Constructor to init boundaries and disance calculation values
    // y = 137500x + 1125 recommended for SHARP
    AnalogIR::AnalogIR(float _minDistance, float _maxDistance, float _m, float _b, gpio::Analog::Pin _pin) : IRSensor(_minDistance, _maxDistance) {
        m = _m;
        b = _b;
        pin = _pin;
    }

    // Initialize ADC
    void AnalogIR::init() {
        gpio::Analog::init();
    }

    // Read sensor and updates current distance
    float AnalogIR::read() {
        // TODO: mV is the analog input reading in mV
        distance = 1 / ((gpio::Analog::Read(pin) - b) / m);
        return getDistance();
    }
} // namespace sensors

} // namespace aruwlib