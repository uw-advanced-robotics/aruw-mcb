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

#ifndef DART_LAUNCHER_SUBSYSTEM_HPP_
#define DART_LAUNCHER_SUBSYSTEM_HPP_

#include <tap/motor/dji_motor.hpp>

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/motor/servo.hpp"

namespace aruwsrc::robot::dart
{
class DartLauncherSubsystem : public tap::control::Subsystem
{
public:
    DartLauncherSubsystem(tap::Drivers *drivers, tap::motor::MotorInterface &pullMotor);

    void initialize() override;
    void moveMotor(int32_t power);
    void refresh() override;

    void refreshSafeDisconnect() override;

    void setSetpoint(float setpoint);
    bool isBeamBroken();
    bool isLimitSwitched();

    const char *getName() const override { return "Dart Launcher Subsystem"; }

protected:
    tap::motor::MotorInterface &motor;
};  // class DartLauncherSubsystem

}  // namespace aruwsrc::robot::dart
#endif  // DART_LAUNCHER_SUBSYSTEM_HPP_
