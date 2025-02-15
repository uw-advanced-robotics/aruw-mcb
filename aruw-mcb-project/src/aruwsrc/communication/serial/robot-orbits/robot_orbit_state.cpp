#include "robot_orbit_state.hpp"

namespace aruwsrc::communication::serial {

void RobotOrbitStateProvider::updateFromVision(tap::communication::serial::RefSerialData::RobotId robotID, const RobotState& state) {
    uint8_t index = static_cast<uint8_t>(robotID);
    if (index < MAX_TRACKED_ROBOTS) {
        visionStates[index] = state;
        visionStates[index].robotId = robotID; 
        hasVisionState[index] = true;
    }
}

void RobotOrbitStateProvider::updateFromAlly(tap::communication::serial::RefSerialData::RobotId robotID, const RobotState& state) {
    uint8_t index = static_cast<uint8_t>(robotID);
    if (index < MAX_TRACKED_ROBOTS && !hasVisionState[index]) {
        allyStates[index] = state;
        allyStates[index].robotId = robotID; 
        hasAllyState[index] = true;
    }
}

bool RobotOrbitStateProvider::getRobotState(tap::communication::serial::RefSerialData::RobotId robotID, RobotState& outState) const {
    uint8_t index = static_cast<uint8_t>(robotID);
    if (index >= MAX_TRACKED_ROBOTS) return false;

    if (hasVisionState[index]) {
        outState = visionStates[index];
        return true;
    } else if (hasAllyState[index]) {
        outState = allyStates[index];
        return true;
    }

    return false;
}

uint8_t RobotOrbitStateProvider::getNumKnownVisionStates(RobotState states[MAX_TRACKED_ROBOTS]) const {
    uint8_t count = 0;
    for (uint8_t i = 0; i < MAX_TRACKED_ROBOTS; i++) {
        if (hasVisionState[i]) {
            states[count++] = visionStates[i];
        }
    }
    return count;
}

} // namespace aruwsrc::communication::serial
