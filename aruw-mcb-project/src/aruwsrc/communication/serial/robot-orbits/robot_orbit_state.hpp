#ifndef ROBOT_ORBIT_STATE_HPP
#define ROBOT_ORBIT_STATE_HPP

#include <cstdint>
#include "tap/communication/serial/ref_serial_data.hpp"

using namespace tap::communication::serial;
namespace aruwsrc::communication::serial {

constexpr uint8_t MAX_TRACKED_ROBOTS = 4;

struct RobotState {
    RefSerialData::RobotId robotId;
    uint8_t xPos;
    uint8_t yPos;
    uint8_t zPos;
};

class RobotOrbitStateProvider {
public:
    void updateFromVision(RefSerialData::RobotId robotID, const RobotState& state);
    void updateFromAlly(RefSerialData::RobotId robotID, const RobotState& state);
    bool getRobotState(RefSerialData::RobotId robotID, RobotState& outState) const;
    uint8_t getNumKnownVisionStates(RobotState states[MAX_TRACKED_ROBOTS]) const;

private:
    RobotState storedStates[MAX_TRACKED_ROBOTS] = {};
    RefSerialData::RobotId storedIDs[MAX_TRACKED_ROBOTS] = {RefSerialData::RobotId::INVALID};
    bool hasState[MAX_TRACKED_ROBOTS] = {false};

    int findRobotIndex(RefSerialData::RobotId robotID) const;
};

} // namespace aruwsrc::communication::serial

#endif // ROBOT_ORBIT_STATE_HPP
