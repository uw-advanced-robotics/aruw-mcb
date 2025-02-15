#include "robot_orbit_subsystem.hpp"

namespace aruwsrc::communication::serial {

RobotOrbitSubsystem::RobotOrbitSubsystem(
    tap::Drivers* drivers,
    aruwsrc::algorithms::odometry::ChassisKFOdometry* chassisOdometry,
    tap::communication::serial::RefSerial* refSerial)
    : tap::control::Subsystem(drivers),
      transmitter(drivers, stateProvider, chassisOdometry, refSerial) {}

void RobotOrbitSubsystem::refresh() {
    static tap::arch::MilliTimeout timeout(100); 

    if (timeout.execute()) {
        transmitter.sendRobotStates();
        timeout.restart(100); 
    }
}

} // namespace aruwsrc::communication::serial
