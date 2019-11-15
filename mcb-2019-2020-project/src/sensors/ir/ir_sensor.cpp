#include "ir_sensor.hpp"

namespace aruwlib {

namespace sensors {
    // Constructor to init boundaries
    IRSensor::IRSensor(float _minDistance, float _maxDistance) {
        minDistance = _minDistance;
        maxDistance = _maxDistance;
    }

    // Get current distance
    float IRSensor::getDistance() {
        return distance;
    }

    // Get minumum distance boundary
    float IRSensor::getMinDistance() {
        return minDistance;
    }

    // Get maximun distance boundary
    float IRSensor::getMaxDistance() {
        return maxDistance;
    }
} // namespace sensors

} // namespace aruwlib