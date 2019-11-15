#include "analog_ir.hpp"

namespace aruwlib {

namespace sensors {
    // Constructor to init boundaries and disance calculation values
    // y = 137500x + 1125 recommended for SHARP
    AnalogIR::AnalogIR(float minDistance, float maxDistance, float m, float b, gpio::Analog::Pin pin): 
        IRSensor(minDistance, maxDistance), m_m(m), m_b(b), m_pin(pin) {}

    // Initialize ADC
    void AnalogIR::init() {
        gpio::Analog::init();
        modm::delayMilliseconds(50); // init delay before first reading
    }

    // Read sensor and updates current distance
    float AnalogIR::read() {
        // TODO: mV is the analog input reading in mV
        //m_distance = 1 / ((gpio::Analog::Read(m_pin) - m_b) / m_m);
        
        m_distance = aruwlib::gpio::Analog::Read(m_pin);
        m_distance /= 1000; // volts
        m_distance = 0.0857 * m_distance - 0.0103;

        m_distance = 1 / m_distance - 0.42;
        
        return getDistance();
    }
} // namespace sensors

} // namespace aruwlib