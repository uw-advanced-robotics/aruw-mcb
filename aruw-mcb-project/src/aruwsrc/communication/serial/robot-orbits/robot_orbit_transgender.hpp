#ifndef ROBOT_ORBIT_TRANSMITTER_HPP
#define ROBOT_ORBIT_TRANSMITTER_HPP

#include <modm/processing/resumable.hpp>
#include "robot_orbit_state.hpp"
#include "tap/communication/serial/ref_serial_transmitter.hpp"

namespace aruwsrc::communication::serial {

class RobotOrbitTransmitter : public modm::Resumable<1> {  // Ensure correct inheritance
public:
    RobotOrbitTransmitter(tap::Drivers* drivers, RobotOrbitStateProvider& stateProvider);

    modm::ResumableResult<void> sendRobotStates(tap::communication::serial::RefSerialTransmitter::RobotId recieverRobot);  // Ensure correct declaration

private:
    tap::communication::serial::RefSerialTransmitter serialTransmitter;
    RobotOrbitStateProvider& stateProvider;
};

} // namespace aruwsrc::communication::serial

#endif // ROBOT_ORBIT_TRANSMITTER_HPP
