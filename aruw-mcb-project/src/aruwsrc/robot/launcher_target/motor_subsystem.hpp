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

#include "aruwsrc/control/motor/tmotor_ak80_9.hpp"

#include "launcher_target_constants.hpp"

namespace aruwsrc::launcher_target
{
class MotorSubsystem : public tap::control::Subsystem
{
public:
    MotorSubsystem(
        tap::Drivers* drivers,
        tap::motor::MotorInterface& motor,
        tap::algorithms::SmoothPidConfig pidConfig)
        : Subsystem(drivers),
          motor(motor),
          velocityPid(pidConfig)
    {
    }

    inline void initialize() override
    {
        this->motor.initialize();
        this->motor.getEncoder()->resetEncoderValue();
    };

    inline void setDesiredRPM(float rpm) { desiredRPM = rpm; }

    inline void refresh() override
    {
        const uint32_t curTime = tap::arch::clock::getTimeMilliseconds();
        const uint32_t dt = curTime - prevTime;
        prevTime = curTime;
        position = getCurrentPosition();

        const float velocityError = desiredRPM - getCurrentRPM();

        velocityPid.runControllerDerivateError(velocityError, dt);

        // safety to make sure we don't go past the bounds
        if ((position < 0 && desiredRPM < 0) ||
            (position > aruwsrc::launcher_target::constants::TARGET_TRAVEL_DISTANCE &&
             desiredRPM > 0))
        {
            stop();
        }
        else
        {
            motor.setDesiredOutput(velocityPid.getOutput());
        }

        // motor.setDesiredOutput(velocityPid.getOutput());

        if (akMotor)
        {
            static_cast<aruwsrc::control::motor::Tmotor_AK809*>(&motor)->sendCanMessage();
        }
    };

    // in output shaft rpm
    inline float getCurrentRPM() const
    {
        return motor.getEncoder()->getVelocity() * 60.0f / M_TWOPI;
    }

    inline float getCurrentPosition()
    {
        return motor.getEncoder()->getPosition().getUnwrappedValue() *
               aruwsrc::launcher_target::constants::WHEEL_DIAMETER / 2.0;
    }

    inline void refreshSafeDisconnect() override { stop(); };

    inline void stop()
    {
        desiredRPM = 0;
        this->motor.setDesiredOutput(0);
        if (akMotor)
        {
            static_cast<aruwsrc::control::motor::Tmotor_AK809*>(&motor)->sendCanMessage();
        }
    }

    const char* getName() const override { return "Motor"; }

private:
    tap::motor::MotorInterface& motor;
    tap::algorithms::SmoothPid velocityPid;

    float desiredRPM{0};
    float position = 0;
    uint32_t prevTime = 0;
    bool akMotor = false;
};

}  // namespace aruwsrc::launcher_target

#endif
