#include "vl6180x.hpp"

namespace aruwlib {

namespace sensors {
    // Constructor to init boundaries
    VL6180X::VL6180X(float minDistance, float maxDistance): IRSensor(minDistance, maxDistance) {}

    // Initialize sensor and I2C
    void VL6180X::init() {
        
    }

    // Read sensor and updates current distance
    float VL6180X::read() {
        return 0;
    }

    // Write 8 bits to the given address
    void VL6180X::write8(uint16_t address, uint8_t data) {
        
    }

    // Write 16 bits to the given address
    void VL6180X::write16(uint16_t address, uint8_t data) {

    }

    // Read 8 bits to the given address
    uint8_t read8(uint16_t address) {
        return 0;
    }

    // Read 16 bits to the given address
    uint16_t read16(uint16_t address) {
        return 0;
    }
} // namespace sensors

} // namespace aruwlib