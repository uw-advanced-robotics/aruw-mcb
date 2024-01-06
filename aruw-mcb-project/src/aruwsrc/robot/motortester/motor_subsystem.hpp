
#ifndef MOTOR_SUBSYSTEM_HPP_
#define MOTOR_SUBSYSTEM_HPP_

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/communication/serial/remote.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/drivers.hpp"
#include "tap/motor/dji_motor.hpp"

namespace aruwsrc::motortester
{

class MotorSubsystem : public tap::control::Subsystem
{
public:
    MotorSubsystem(
        tap::Drivers* drivers,
        tap::motor::MotorInterface& motor,
        tap::algorithms::SmoothPidConfig pidConfig,
        float gearRatio)
        : Subsystem(drivers),
          motor(motor),
          velocityPid(pidConfig),
          gearRatio(gearRatio)
    {
    }

    inline void initialize() override { this->motor.initialize(); };

    inline void setDesiredRPM(float rpm) { desiredRPM = rpm; }

    inline void refresh() override
    {
        const uint32_t curTime = tap::arch::clock::getTimeMilliseconds();
        const uint32_t dt = curTime - prevTime;
        prevTime = curTime;

        const float velocityError = desiredRPM - getCurrentRPM();

        velocityPid.runControllerDerivateError(velocityError, dt);

        motor.setDesiredOutput(velocityPid.getOutput());
    };

    // in output shaft rpm
    inline float getCurrentRPM() const { return (motor.getShaftRPM() * gearRatio); }

    inline void refreshSafeDisconnect() override { stop(); };

    inline void stop()
    {
        desiredRPM = 0;
        this->motor.setDesiredOutput(0);
    }

    const char* getName() override { return "Motor"; }

private:
    tap::motor::MotorInterface& motor;
    tap::algorithms::SmoothPid velocityPid;
    float gearRatio;

    float desiredRPM{0};
    uint32_t prevTime = 0;
};

}  // namespace aruwsrc::motortester

#endif
