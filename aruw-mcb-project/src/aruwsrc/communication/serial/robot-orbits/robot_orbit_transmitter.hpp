#ifndef ROBOT_ORBIT_TRANSMITTER_HPP
#define ROBOT_ORBIT_TRANSMITTER_HPP

#include <modm/processing/resumable.hpp>
#include "robot_orbit_state.hpp"
#include "tap/communication/serial/ref_serial.hpp"
#include "tap/communication/serial/ref_serial_transmitter.hpp"
#include "tap/control/subsystem.hpp"
#include "aruwsrc/algorithms/odometry/chassis_kf_odometry.hpp"

using namespace aruwsrc::algorithms::odometry;
using namespace tap::communication::serial;

namespace aruwsrc::communication::serial {

class RobotOrbitTransmitter : public RefSerial::RobotToRobotMessageHandler {
public:
    RobotOrbitTransmitter(
        tap::Drivers* drivers, 
        RobotOrbitStateProvider& stateProvider, 
        ChassisKFOdometry* chassisOdometry, 
        RefSerial* refSerial
    );

    void sendRobotStates();
    void parseIncomingMessage(const DJISerial::ReceivedSerialMessage& message);
    void operator()(
        const DJISerial::ReceivedSerialMessage &message) override final;

private:
    tap::Drivers* drivers;
    RefSerialTransmitter serialTransmitter;
    RobotOrbitStateProvider& stateProvider;
    ChassisKFOdometry* odometry;
    RefSerial* refSerial;

    RefSerialTransmitter::RobotId getAllyRobotId() const;

    constexpr static uint8_t STATIC_CAST_SCALE_FACTOR = 100;

};

} // namespace aruwsrc::communication::serial

#endif // ROBOT_ORBIT_TRANSMITTER_HPP
