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

#ifndef RAW_MOTOR_SUBSYSTEM_HPP_
#define RAW_MOTOR_SUBSYSTEM_HPP_

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/communication/serial/remote.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/drivers.hpp"
#include "tap/motor/dji_motor.hpp"

#include "aruwsrc/control/motor/tmotor_ak80_9.hpp"

namespace aruwsrc::characterizer
{
class RawMotorSubsystem : public tap::control::Subsystem
{
public:
    RawMotorSubsystem(
        tap::Drivers* drivers,
        tap::motor::MotorInterface& motor,
        bool akMotor = false)
        : Subsystem(drivers),
          motor(motor),
          akMotor(akMotor)
    {
    }

    inline void initialize() override { this->motor.initialize(); };

    inline void setDesiredOutput(int32_t rpm) { desiredOutput = rpm; }

    inline void refresh() override
    {
        motor.setDesiredOutput(desiredOutput);
        if (akMotor)
        {
            static_cast<aruwsrc::control::motor::Tmotor_AK809*>(&motor)->sendCanMessage();
        }
    };

    inline void refreshSafeDisconnect() override { stop(); };

    inline void stop()
    {
        desiredOutput = 0;
        this->motor.setDesiredOutput(0);
        if (akMotor)
        {
            static_cast<aruwsrc::control::motor::Tmotor_AK809*>(&motor)->sendCanMessage();
        }
    }

    const char* getName() const override { return "Motor"; }

private:
    tap::motor::MotorInterface& motor;

    int32_t desiredOutput{0};
    bool akMotor;
};

}  // namespace aruwsrc::characterizer

#endif
