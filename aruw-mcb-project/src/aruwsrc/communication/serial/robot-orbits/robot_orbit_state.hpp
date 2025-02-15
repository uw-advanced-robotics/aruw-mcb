#ifndef ROBOT_ORBIT_STATE_HPP
#define ROBOT_ORBIT_STATE_HPP

#include <cstdint>
#include "tap/communication/serial/ref_serial_data.hpp"

namespace aruwsrc::communication::serial {

constexpr uint8_t MAX_TRACKED_ROBOTS = 4;

struct RobotState {
    tap::communication::serial::RefSerialData::RobotId robotId; 
    uint8_t xPos;
    uint8_t yPos;
    uint8_t zPos;
};

class RobotOrbitStateProvider {
public:
    void updateFromVision(tap::communication::serial::RefSerialData::RobotId robotID, const RobotState& state);
    void updateFromAlly(tap::communication::serial::RefSerialData::RobotId robotID, const RobotState& state);
    bool getRobotState(tap::communication::serial::RefSerialData::RobotId robotID, RobotState& outState) const;
    uint8_t getNumKnownVisionStates(RobotState states[MAX_TRACKED_ROBOTS]) const;

private:
    RobotState visionStates[MAX_TRACKED_ROBOTS] = {};
    RobotState allyStates[MAX_TRACKED_ROBOTS] = {};
    bool hasVisionState[MAX_TRACKED_ROBOTS] = {};
    bool hasAllyState[MAX_TRACKED_ROBOTS] = {};
};

} // namespace aruwsrc::communication::serial

#endif // ROBOT_ORBIT_STATE_HPP
