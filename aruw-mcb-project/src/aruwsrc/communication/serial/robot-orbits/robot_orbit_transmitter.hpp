#ifndef ROBOT_ORBIT_TRANSMITTER_HPP
#define ROBOT_ORBIT_TRANSMITTER_HPP

#include <modm/processing/resumable.hpp>
#include "robot_orbit_state.hpp"
#include "tap/communication/serial/ref_serial.hpp"
#include "tap/communication/serial/ref_serial_transmitter.hpp"
#include "tap/control/subsystem.hpp"
#include "aruwsrc/algorithms/odometry/chassis_kf_odometry.hpp"

namespace aruwsrc::communication::serial {

class RobotOrbitTransmitter {
public:
    RobotOrbitTransmitter(
        tap::Drivers* drivers, 
        RobotOrbitStateProvider& stateProvider, 
        aruwsrc::algorithms::odometry::ChassisKFOdometry* chassisOdometry, 
        tap::communication::serial::RefSerial* refSerial
    );

    void sendRobotStates();

private:
    tap::Drivers* drivers;
    tap::communication::serial::RefSerialTransmitter serialTransmitter;
    RobotOrbitStateProvider& stateProvider;
    aruwsrc::algorithms::odometry::ChassisKFOdometry* odometry;
    tap::communication::serial::RefSerial* refSerial;

    tap::communication::serial::RefSerialTransmitter::RobotId getAllyRobotId() const;
};

} // namespace aruwsrc::communication::serial

#endif // ROBOT_ORBIT_TRANSMITTER_HPP
