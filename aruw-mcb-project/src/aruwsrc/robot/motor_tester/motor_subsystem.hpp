/*
 * Copyright (c) 2023-2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruw-mcb.
 *
 * aruw-mcb is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aruw-mcb is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aruw-mcb.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef MOTOR_SUBSYSTEM_HPP_
#define MOTOR_SUBSYSTEM_HPP_

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/communication/serial/remote.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/drivers.hpp"
#include "tap/motor/dji_motor.hpp"

namespace aruwsrc::motor_tester
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

    const char* getName() const override { return "Motor"; }

private:
    tap::motor::MotorInterface& motor;
    tap::algorithms::SmoothPid velocityPid;
    float gearRatio;

    float desiredRPM{0};
    uint32_t prevTime = 0;
};

}  // namespace aruwsrc::motor_tester

#endif
