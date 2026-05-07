/*
 * Copyright (c) 2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef SERVO_SUBSYSTEM_HPP_
#define SERVO_SUBSYSTEM_HPP_

#include "tap/control/subsystem.hpp"

#include "aruwsrc/communication/mcb-lite/mcb_lite.hpp"
#include "aruwsrc/communication/mcb-lite/motor/virtual_servo.hpp"

namespace aruwsrc::engineer
{
class ServoSubsystem : public tap::control::Subsystem
{
public:
    ServoSubsystem(tap::Drivers* drivers, aruwsrc::communication::mcb_lite::MCBLite& mcbLite)
        : tap::control::Subsystem(drivers),
          servoOne(drivers, tap::gpio::Pwm::Pin::X, 1.0f, 0.0f, 0.01f, mcbLite.pwm),
          servoTwo(drivers, tap::gpio::Pwm::Pin::Buzzer, 1.0f, 0.0f, 0.01f, mcbLite.pwm),
          mcbLite(mcbLite)
    {
    }

    void refresh() override
    {
        servoOne.updateSendPwmRamp();
        servoTwo.updateSendPwmRamp();
    }

    void refreshSafeDisconnect() override
    {
        mcbLite.pwm.write(0.0f, servoOne.getPin());
        mcbLite.pwm.write(0.0f, servoTwo.getPin());
    }

    aruwsrc::communication::mcb_lite::motor::VirtualServo servoOne;
    aruwsrc::communication::mcb_lite::motor::VirtualServo servoTwo;
    aruwsrc::communication::mcb_lite::MCBLite& mcbLite;
};
}  // namespace aruwsrc::engineer
#endif