#include "robot_orbit_state.hpp"

namespace aruwsrc::communication::serial {

int RobotOrbitStateProvider::findRobotIndex(RefSerialData::RobotId robotID) const {
    for (uint8_t i = 0; i < MAX_TRACKED_ROBOTS; i++) {
        if (hasState[i] && storedIDs[i] == robotID) {
            return i;
        }
    }
    return -1;
}

void RobotOrbitStateProvider::updateFromVision(RefSerialData::RobotId robotID, const RobotState& state) {
    int index = findRobotIndex(robotID);
    if (index != -1) {
        storedStates[index] = state;
    } else {
        for (uint8_t i = 0; i < MAX_TRACKED_ROBOTS; i++) {
            if (!hasState[i]) {
                storedStates[i] = state;
                storedIDs[i] = robotID;
                hasState[i] = true;
                return;
            }
        }
    }
}

void RobotOrbitStateProvider::updateFromAlly(RefSerialData::RobotId robotID, const RobotState& state) {
    int index = findRobotIndex(robotID);
    if (index != -1) {
        storedStates[index] = state;
    } else {
        for (uint8_t i = 0; i < MAX_TRACKED_ROBOTS; i++) {
            if (!hasState[i]) {
                storedStates[i] = state;
                storedIDs[i] = robotID;
                hasState[i] = true;
                return;
            }
        }
    }
}

bool RobotOrbitStateProvider::getRobotState(RefSerialData::RobotId robotID, RobotState& outState) const {
    int index = findRobotIndex(robotID);
    if (index != -1) {
        outState = storedStates[index];
        return true;
    }
    return false;
}

uint8_t RobotOrbitStateProvider::getNumKnownVisionStates(RobotState states[MAX_TRACKED_ROBOTS]) const {
    uint8_t count = 0;
    for (uint8_t i = 0; i < MAX_TRACKED_ROBOTS; i++) {
        if (hasState[i]) {
            states[count++] = storedStates[i];
        }
    }
    return count;
}

} // namespace aruwsrc::communication::serial
