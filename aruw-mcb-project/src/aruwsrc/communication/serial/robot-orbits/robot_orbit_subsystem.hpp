#ifndef ROBOT_ORBIT_SUBSYSTEM_HPP
#define ROBOT_ORBIT_SUBSYSTEM_HPP

#include "tap/control/subsystem.hpp"
#include "robot_orbit_transmitter.hpp"
#include "tap/architecture/timeout.hpp"

namespace aruwsrc::communication::serial {

class RobotOrbitSubsystem : public tap::control::Subsystem {
public:
    explicit RobotOrbitSubsystem(
        tap::Drivers* drivers,
        aruwsrc::algorithms::odometry::ChassisKFOdometry* chassisOdometry,
        tap::communication::serial::RefSerial* refSerial);
    void refresh() override;

    inline void updateFromVision(tap::communication::serial::RefSerialData::RobotId robotID, const RobotState& state) {
        stateProvider.updateFromVision(robotID, state);
    }

    inline void updateFromAlly(tap::communication::serial::RefSerialData::RobotId robotID, const RobotState& state) {
        stateProvider.updateFromAlly(robotID, state);
    }

private:
    RobotOrbitStateProvider stateProvider;
    RobotOrbitTransmitter transmitter;
};

} // namespace aruwsrc::communication::serial

#endif // ROBOT_ORBIT_SUBSYSTEM_HPP
