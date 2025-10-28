#ifndef STICK_TORQUE_COMMAND_HPP_
#define STICK_TORQUE_COMMAND_HPP_

#include "tap/control/command.hpp"

#include "motor_subsystem.hpp"
#include "tap/communication/serial/remote.hpp"


namespace aruwsrc::robot::motor_tester {

class StickTorqueCommand : public tap::control::Command {
public:
    StickTorqueCommand(MotorSubsystem& motor, 
                       tap::communication::serial::Remote& remote, 
                       tap::communication::serial::Remote::Channel channel, 
                       float sensitivity) 
        : motor(motor), remote(remote), channel(channel), sensitivity(sensitivity) {}

    void execute() override {
        int32_t input = remote.getChannel(channel) * sensitivity 
            * tap::motor::DjiMotor::MAX_OUTPUT_C620;

        motor.setDesiredOutput(input);
    }

private:
    MotorSubsystem&  motor;
    const tap::communication::serial::Remote& remote;
    const tap::communication::serial::Remote::Channel channel;
    const float sensitivity;
};

} // namespace aruwsrc::robot::motor_tester

#endif // STICK_TORQUE_COMMAND_HPP_