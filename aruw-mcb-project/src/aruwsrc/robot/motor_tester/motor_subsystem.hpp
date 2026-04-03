#ifndef MOTOR_SUBSYSTEM_HPP_
#define MOTOR_SUBSYSTEM_HPP_

#include "tap/control/subsystem.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/algorithms/smooth_pid.hpp"

namespace aruwsrc::motor_tester {

class MotorSubsystem : public tap::control::Subsystem
{
public:
    inline MotorSubsystem(
        tap::Drivers* drivers,
        tap::motor::MotorInterface& motor,
        tap::algorithms::SmoothPidConfig pidConfig)
        : Subsystem(drivers),
          motor(motor),
          pid(pidConfig)
    {
    };

    void initialize() override {motor.initialize();}

    float getDesiredOutput() const { return desiredOutput; }
    void setDesiredOutput(float desiredOutput)
    {
        if (desiredOutput > tap::motor::DjiMotor::MAX_OUTPUT_C620) {
            desiredOutput = tap::motor::DjiMotor::MAX_OUTPUT_C620;
        } else if (desiredOutput < -tap::motor::DjiMotor::MAX_OUTPUT_C620) {
            desiredOutput = -tap::motor::DjiMotor::MAX_OUTPUT_C620;
        }
        this->desiredOutput = desiredOutput;
        motor.setDesiredOutput(static_cast<int32_t>(desiredOutput));
    }

    void refresh() override
    {
        float motorPosition = motor.getEncoder()->getPosition().getUnwrappedValue();
        float positionError = desiredPosition - motorPosition;

        //desiredOutput = pid.runController(positionError, -motor.getEncoder()->getVelocity(), 0.002f);

        //motor.setDesiredOutput(desiredOutput);
    }

    float getDesiredPosition() const { return desiredPosition; }
    void setDesiredPosition(float desiredPosition) { this->desiredPosition = desiredPosition; }


    


private:

    float desiredOutput = 0;
    float desiredPosition = 0;
    tap::motor::MotorInterface& motor;
    tap::algorithms::SmoothPid pid;
};
    


}

#endif // MOTOR_SUBSYSTEM_HPP_