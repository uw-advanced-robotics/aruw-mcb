#ifndef MOTOR_SUBSYSTEM_HPP_
#define MOTOR_SUBSYSTEM_HPP_

#include "tap/control/subsystem.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/algorithms/smooth_pid.hpp"
#include "tap/drivers.hpp"
#include "tap/communication/serial/remote.hpp"


namespace aruwsrc::motor_tester {

class MotorSubsystem : public tap::control::Subsystem
{
public:
    MotorSubsystem(
        tap::Drivers* drivers,
        tap::motor::MotorInterface& motor,
        tap::algorithms::SmoothPidConfig pidConfig)
        : Subsystem(drivers),
          motor(motor),
          pid(pidConfig)
    {
    };

    void initialize() override {motor.initialize();}

   
    void setDesiredRPM(float rpm){desiredRPM = rpm;}

    void setDesiredOutput(float desiredOutput)
    {
        this->desiredOutput = desiredOutput;
        motor.setDesiredOutput(static_cast<int32_t>(desiredOutput));
    }

    void refresh() override
    {
        const uint32_t currentTime = tap::arch::clock::getTimeMilliseconds();
        const uint32_t dt = currentTime - prevTime;
        prevTime = currentTime;

        const float velocityError = desiredRPM - motor.getEncoder()->getVelocity()*60.0f / M_TWOPI;
        pid.runControllerDerivateError(velocityError, dt);
        motor.setDesiredOutput(pid.getOutput());
    }

    void stop()
    {
        desiredRPM = 0;
        this->motor.setDesiredOutput(0);
    }

    


private:

    float desiredRPM = 0;
    float desiredOutput = 0;
    uint32_t prevTime = 0;
    tap::motor::MotorInterface& motor;
    tap::algorithms::SmoothPid pid;
};
    


}

#endif // MOTOR_SUBSYSTEM_HPP_