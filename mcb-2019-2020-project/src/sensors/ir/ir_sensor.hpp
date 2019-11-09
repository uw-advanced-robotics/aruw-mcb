/*
General IR sensor class for basic analog IR or other sensors to build off of
Planned sensors: basic analog IR, Adafruit VL6180X, and Seeed IR
*/

#ifndef IRSENSOR_H
#define IRSENSOR_H

#include "rm-dev-board-a/board.hpp"

using namespace modm::literals;

namespace aruwlib {

namespace sensors {

class IRSensor {
 public:
    // Constructor to init boundaries
    IRSensor(float minDistance, float maxDistance);

    // Init sensor
    virtual void init();

    // Read sensor and updates current distance
    virtual float read(); 

    // Get current distance
    float getDistance();

    // Get minumum distance boundary
    float getMinDistance();

    // Get maximun distance boundary
    float getMaxDistance();

 protected:
    // Distance from sensor
    float distance;

    // Lower boundary for reliable readings
    float minDistance;

    // Upper boundary for reliable readings
    float maxDistance;
};

} // namespace sensors

} // namespace aruwlib

#endif // IRSENSOR_H