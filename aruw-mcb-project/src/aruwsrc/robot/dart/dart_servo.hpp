/*
 * Copyright (c) 2024-2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef DART_SERVO_HPP_
#define DART_SERVO_HPP_

#include <tap/motor/dji_motor.hpp>

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/motor/servo.hpp"

namespace aruwsrc::dart
{
class DartServo : public tap::control::Subsystem
{
public:
    DartServo(tap::Drivers* drivers);

    void initialize() override;
    void refresh() override;

    void refreshSafeDisconnect() override;

    // set servo to the open angle
    void setOpen();

    // set servo to the close angle
    void setClose();

    // return the angle defined as open as a PWM value
    float getOpenPWM();

    // return the angle defined as close as a PWM value
    float getClosePWM();

    tap::motor::Servo& getServo() { return servo; }

    const char* getName() const override { return "Dart Servo"; }

protected:
    tap::motor::Servo servo;

};  // class DartServo

}  // namespace aruwsrc::dart
#endif  // DART_SERVO_HPP_
