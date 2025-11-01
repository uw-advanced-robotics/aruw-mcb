#ifndef STICK_TORQUE_COMMAND_HPP_
#define STICK_TORQUE_COMMAND_HPP_

#include "tap/control/command.hpp"

#include "motor_subsystem.hpp"
#include "tap/communication/serial/remote.hpp"


namespace aruwsrc::motor_tester {

class StickTorqueCommand : public tap::control::Command {
public:
    StickTorqueCommand(MotorSubsystem& motor_subsystem, 
                       tap::communication::serial::Remote& remote, 
                       tap::communication::serial::Remote::Channel channel, 
                       float sensitivity) 
        : motor_subsystem(motor_subsystem), remote(remote), channel(channel), sensitivity(sensitivity) {
            addSubsystemRequirement(&motor_subsystem);
        }

    void execute() override {
        int32_t input = remote.getChannel(channel) * sensitivity 
            * tap::motor::DjiMotor::MAX_OUTPUT_C620;

        motor_subsystem.setTargetVelocity(input);
    }

    bool isReady() override {
        return !isFinished() && motor_subsystem.isOnline();
    }

    bool isFinished() const override {
        return !motor_subsystem.isOnline();
    }

    void end(bool) override {
        motor_subsystem.setTargetVelocity(0);
        motor_subsystem.initialize();
    }

    void initialize() override {
        motor_subsystem.initialize();
    }

    char* getName() const override {
        return "test_motor";
    }

private:
    MotorSubsystem& motor_subsystem;
    const tap::communication::serial::Remote& remote;
    const tap::communication::serial::Remote::Channel channel;
    const float sensitivity;
};

} // namespace aruwsrc::robot::motor_tester

#endif // STICK_TORQUE_COMMAND_HPP_