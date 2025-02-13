#ifndef ROBOT_ORBIT_STATE_HPP
#define ROBOT_ORBIT_STATE_HPP

#include <cstdint>
#include "tap/communication/serial/ref_serial_transmitter.hpp"

namespace aruwsrc::communication::serial {

constexpr uint8_t MAX_TRACKED_ROBOTS = 4;

enum class RobotId : uint8_t {
    SELF = 0,
    ENEMY_STANDARD = 1,
    ENEMY_HERO = 2,
    ENEMY_SENTRY = 3,
    INVALID = 255
};

struct RobotState {
    uint8_t plateID;
    uint8_t xPos;
    uint8_t yPos;
    uint8_t zPos;
};

class RobotOrbitStateProvider {
public:
    void updateFromVision(RobotId robotID, const RobotState& state);
    void updateFromAlly(RobotId robotID, const RobotState& state);
    bool getRobotState(RobotId robotID, RobotState& outState) const;
    uint8_t getKnownStates(RobotState states[MAX_TRACKED_ROBOTS]) const;

private:
    RobotState visionStates[MAX_TRACKED_ROBOTS] = {};
    RobotState allyStates[MAX_TRACKED_ROBOTS] = {};
    bool hasVisionState[MAX_TRACKED_ROBOTS] = {};
    bool hasAllyState[MAX_TRACKED_ROBOTS] = {};
};

} // namespace aruwsrc::communication::serial

#endif // ROBOT_ORBIT_STATE_HPP
