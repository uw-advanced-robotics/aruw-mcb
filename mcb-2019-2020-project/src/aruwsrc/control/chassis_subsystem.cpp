#include "src/aruwsrc/control/chassis_subsystem.hpp"

namespace aruwsrc
{

namespace control
{
    const aruwlib::motor::MotorId ChassisSubsystem::LEFT_TOP_MOTOR_ID = aruwlib::motor::MOTOR4;
    const aruwlib::motor::MotorId ChassisSubsystem::LEFT_BOT_MOTOR_ID = aruwlib::motor::MOTOR5;
    const aruwlib::motor::MotorId ChassisSubsystem::RIGHT_TOP_MOTOR_ID = aruwlib::motor::MOTOR6;
    const aruwlib::motor::MotorId ChassisSubsystem::RIGHT_BOT_MOTOR_ID = aruwlib::motor::MOTOR7;

    void ChassisSubsystem::setDesiredOutput(float x, float y, float r)
    {
        // TODO calculate motor rpm values based on given x, y, and r
        leftTopRpm = 0;
        leftBotRpm = 0;
        rightTopRpm = 0;
        rightBotRpm = 0;
    }

    void ChassisSubsystem::refresh()
    {
        updateMotorRpmPid(&leftTopVelocityPid, &leftTopMotor, leftTopRpm);
        updateMotorRpmPid(&leftBotVelocityPid, &leftBotMotor, leftBotRpm);
        updateMotorRpmPid(&rightTopVelocityPid, &rightTopMotor, rightTopRpm);
        updateMotorRpmPid(&rightBotVelocityPid, &rightBotMotor, rightBotRpm);
    }

    void ChassisSubsystem::updateMotorRpmPid(
        modm::Pid<float>* pid,
        aruwlib::motor::DjiMotor* const motor,
        float desiredRpm
    ) {
        pid->update(desiredRpm - motor->getShaftRPM());
        motor->setDesiredOutput(pid->getValue());
    }
}  // namespace control

}  // namespace aruwsrc