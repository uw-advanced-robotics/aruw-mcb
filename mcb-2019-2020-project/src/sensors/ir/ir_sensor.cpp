#include "ir_sensor.hpp"

namespace aruwlib {

namespace sensors {
    // Constructor to init boundaries
    IRSensor::IRSensor(float minDistance, float maxDistance):
        m_minDistance(minDistance), 
        m_maxDistance(maxDistance) {}

    // Destructor
    IRSensor::~IRSensor() {}

    // Get current distance
    float IRSensor::getDistance() {
        return m_distance;
    }

    // Get minumum distance boundary
    float IRSensor::getMinDistance() {
        return m_minDistance;
    }

    // Get maximun distance boundary
    float IRSensor::getMaxDistance() {
        return m_maxDistance;
    }
} // namespace sensors

} // namespace aruwlib