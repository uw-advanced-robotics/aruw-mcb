/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef DART_SUBSYSTEM_HPP_
#define DART_SUBSYSTEM_HPP_

#include "limits.h"

#include "tap/control/subsystem.hpp"
#include "tap/motor/dji_motor.hpp"

#define WINDUP_SPEED = SHRT_MAX / 2

namespace aruwsrc::dart
{
class DartSubsystem : public tap::control::Subsystem
{
public:
    DartSubsystem(tap::Drivers* drivers, tap::motor::DjiMotor* motor);

    const char* getName() const override { return "Dart"; }

    void windUp();

    void refreshSafeDisconnect() override { stop(); }

    void stop();

private:
    tap::motor::DjiMotor* motor;
};

}  // namespace aruwsrc::dart

#endif  // DART_SUBSYSTEM_HPP_
