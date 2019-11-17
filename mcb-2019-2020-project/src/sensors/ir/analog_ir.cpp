#include "analog_ir.hpp"

namespace aruwlib {

namespace sensors {
    // Constructor to init boundaries
    // By default set m/b/offset values to SHARP IR and analog pin to S
    AnalogIR::AnalogIR(float minDistance, float maxDistance):
        IRSensor(minDistance, maxDistance),
        m_m(SHARPIR_m),
        m_b(SHARPIR_b),
        m_offset(SHARPIR_offset),
        m_pin(gpio::Analog::Pin::S) {}

    // Constructor to init boundaries and analog pin
    // By default set m/b/offset values to SHARP IR
    AnalogIR::AnalogIR(float minDistance, float maxDistance, gpio::Analog::Pin pin):
        IRSensor(minDistance, maxDistance),
        m_m(SHARPIR_m),
        m_b(SHARPIR_b),
        m_offset(SHARPIR_offset),
        m_pin(pin) {}

    // Constructor to init boundaries, disance calculation values, and analog pin
    AnalogIR::AnalogIR(float minDistance, float maxDistance, float m, float b, float offset, gpio::Analog::Pin pin): 
        IRSensor(minDistance, maxDistance),
        m_m(m),
        m_b(b),
        m_offset(offset),
        m_pin(pin) {}

    // Initialize ADC
    void AnalogIR::init() {
        gpio::Analog::init();

        // init delay before first reading
        modm::delayMilliseconds(50);
    }

    // Read sensor and update current distance
    float AnalogIR::read() {
        // Read analog pin and convert to volts
        m_distance = aruwlib::gpio::Analog::Read(m_pin);
        m_distance /= 1000;

        // Linear model
        m_distance = m_m * m_distance + m_b;

        // Convert to cm distance
        m_distance = 1 / m_distance + m_offset;
        
        return getDistance();
    }
} // namespace sensors

} // namespace aruwlib