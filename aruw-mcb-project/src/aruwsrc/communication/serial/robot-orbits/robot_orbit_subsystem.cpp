#include "robot_orbit_subsystem.hpp"

namespace aruwsrc::communication::serial {

RobotOrbitSubsystem::RobotOrbitSubsystem(tap::Drivers* drivers,
    tap::communication::serial::RefSerialTransmitter::RobotId allyRobot)
    : tap::control::Subsystem(drivers),
      transmitter(drivers, stateProvider),
      allyRobot(allyRobot) {}

void RobotOrbitSubsystem::refresh() {
    (void)transmitter.sendRobotStates(allyRobot);
}

} // namespace aruwsrc::communication::serial
