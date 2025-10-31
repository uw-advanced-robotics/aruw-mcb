#include "stick_torque_command.hpp"
#include "tap/motor/dji_motor.hpp"

aruwsrc::motor_tester::StickTorqueCommand::StickTorqueCommand(
    tap::Drivers* drivers,
    MotorSubsystem* motorSubsystem,
    tap::communication::serial::Remote::Channel channel,
    float scale) : drivers(drivers), motorSubsystem(motorSubsystem), channel(channel), prevTime(0), scale(scale) {
    
}

bool aruwsrc::motor_tester::StickTorqueCommand::isReady() { return !isFinished(); }

void aruwsrc::motor_tester::StickTorqueCommand::initialize() {
    this->motorSubsystem->initialize();
    prevTime = tap::arch::clock::getTimeMilliseconds();
}

void aruwsrc::motor_tester::StickTorqueCommand::execute() {
    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t dt = currTime - prevTime;
    prevTime = currTime;

    float val = drivers->remote.getChannel(channel);

    int32_t input = static_cast<int32_t>(static_cast<float>(tap::motor::DjiMotor::MAX_OUTPUT_C620) * val);
    motorSubsystem->setDesiredOutput(input);
}

bool aruwsrc::motor_tester::StickTorqueCommand::isFinished() const { 
    return !motorSubsystem->isOnline();
}

void aruwsrc::motor_tester::StickTorqueCommand::end(bool interrupted) {
    motorSubsystem->setDesiredOutput(0);
}
