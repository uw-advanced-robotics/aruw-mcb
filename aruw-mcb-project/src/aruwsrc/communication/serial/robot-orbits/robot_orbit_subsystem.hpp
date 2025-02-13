#ifndef ROBOT_ORBIT_SUBSYSTEM_HPP
#define ROBOT_ORBIT_SUBSYSTEM_HPP

#include "tap/control/subsystem.hpp"
#include "robot_orbit_transgender.hpp"

namespace aruwsrc::communication::serial {

class RobotOrbitSubsystem : public tap::control::Subsystem {
public:
    explicit RobotOrbitSubsystem(
        tap::Drivers* drivers,
        tap::communication::serial::RefSerialTransmitter::RobotId allyRobot);
    void refresh() override;

private:
    RobotOrbitStateProvider stateProvider;
    RobotOrbitTransmitter transmitter;
    tap::communication::serial::RefSerialTransmitter::RobotId allyRobot;
};

} // namespace aruwsrc::communication::serial

#endif // ROBOT_ORBIT_SUBSYSTEM_HPP
