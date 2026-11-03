#include "stick_torque_command.hpp"
#include "tap/communication/serial/remote.hpp"
#include "motor_subsystem.hpp"
#include "tap/motor/dji_motor.hpp"

using namespace tap::communication::serial;

namespace aruwsrc::motor_tester
{
StickTorqueCommand::StickTorqueCommand(
    tap::Drivers* drivers,
    Remote::Channel channel,
    motor_tester::MotorSubsystem* motor,
    float scaleFactor)
    : drivers(drivers),
      channel(channel),
      motor(motor),
      scaleFactor(scaleFactor)
    {
        addSubsystemRequirement(motor);
    }

    bool StickTorqueCommand::isReady() {
        return true;
    }

    void StickTorqueCommand::initialize() {

    }

    void StickTorqueCommand::execute() {
        float input = drivers->remote.getChannel(channel);
        motor->setDesiredOutput(input * tap::motor::DjiMotor::MAX_OUTPUT_C620 * scaleFactor);
    }

    bool StickTorqueCommand::isFinished() const {
        return false;
    }

    void StickTorqueCommand::end(bool) {
        motor->setDesiredOutput(0);
    }
}