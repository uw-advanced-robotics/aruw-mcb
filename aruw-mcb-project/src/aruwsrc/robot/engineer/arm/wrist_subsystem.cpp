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

#include "aruwsrc/robot/engineer/arm/wrist_subsystem.hpp"

namespace aruwsrc
{
namespace engineer
{
WristSubsystem::WristSubsystem(
    tap::Drivers *drivers,
    tap::motor::MotorInterface &motorLeft,
    tap::motor::MotorInterface &motorRight,
    tap::algorithms::SmoothPidConfig configPitch,
    tap::algorithms::SmoothPidConfig configYaw,
    float ratio,
    float kS,
    float epsilon)
    : tap::control::Subsystem(drivers),
      motorLeft(motorLeft),
      motorRight(motorRight),
      pidPitch(configPitch),
      pidYaw(configYaw),
      ratio(ratio),
      kS(kS),
      epsilon(epsilon)
{
    setpointPitch = 0;
    setpointYaw = 0;
}

float WristSubsystem::getPitch() { return 0.0; }  // todo logic

float WristSubsystem::getYaw() { return 0.0; }

bool WristSubsystem::atSetpoint()
{
    return tap::algorithms::compareFloatClose(setpointPitch, getPitch(), epsilon) &&
           tap::algorithms::compareFloatClose(setpointYaw, getYaw(), epsilon);
}

void WristSubsystem::refresh() {}

void WristSubsystem::refreshSafeDisconnect()
{
    motorLeft.setDesiredOutput(0);
    motorRight.setDesiredOutput(0);
}
}  // namespace engineer
}  // namespace aruwsrc