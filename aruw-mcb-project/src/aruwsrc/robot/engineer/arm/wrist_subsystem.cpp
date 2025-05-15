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
    tap::encoder::EncoderInterface &encoderPitch,
    tap::encoder::EncoderInterface &encoderYaw,
    const tap::algorithms::SmoothPidConfig configPitch,
    const tap::algorithms::SmoothPidConfig configYaw,
    float minPitch,
    float maxPitch,
    float minYaw,
    float maxYaw,
    float ratio,
    float kS,
    float epsilon)
    : tap::control::Subsystem(drivers),
      motorLeft(motorLeft),
      motorRight(motorRight),
      encoderPitch(encoderPitch),
      encoderYaw(encoderYaw),
      pidPitch(configPitch),
      pidYaw(configYaw),
      minPitch(minPitch),
      maxPitch(maxPitch),
      minYaw(minYaw),
      maxYaw(maxYaw),
      ratio(ratio),
      kS(kS),
      epsilon(epsilon)
{
    setpointPitch = 0;
    setpointYaw = 0;
}

void WristSubsystem::setSetpointPitch(float setpoint)
{
    if(minPitch == maxPitch)
        setpointPitch = setpoint;
    else 
        setpointPitch = std::clamp(setpoint, minPitch, maxPitch);
}

void WristSubsystem::setSetpointYaw(float setpoint)
{
    if(minYaw == maxYaw)
        setpointYaw = setpoint;
    else
        setpointYaw = std::clamp(setpoint, minYaw, maxYaw);
}

float WristSubsystem::getPitch() { return encoderPitch.getPosition().getUnwrappedValue(); }

float WristSubsystem::getYaw() { return encoderYaw.getPosition().getUnwrappedValue(); }

bool WristSubsystem::atSetpoint()
{
    return tap::algorithms::compareFloatClose(setpointPitch, getPitch(), epsilon) &&
           tap::algorithms::compareFloatClose(setpointYaw, getYaw(), epsilon);
}

void WristSubsystem::initialize()
{
    motorLeft.initialize();
    motorRight.initialize();
    encoderPitch.initialize();
    encoderYaw.initialize();
}

float OutPitch;
float OutYaw;
float OutLeft;
float OutRight;

void WristSubsystem::refresh()
{
    float outPitch =
        pidPitch.runController(setpointPitch - getPitch(), encoderPitch.getVelocity(), 2.0f);
    float outYaw =
        pidYaw.runController(setpointYaw - getYaw(), encoderYaw.getVelocity(), 2.0f);  // todo ks

        OutPitch = outPitch;
        OutYaw = outYaw;
    float outLeft = ratio * outYaw + outPitch;
    float outRight = ratio * outYaw - outPitch;  // todo derived from info in notion, double check

    OutLeft = outLeft;
    OutRight = outRight;
    motorLeft.setDesiredOutput(outLeft + kS);
    motorRight.setDesiredOutput(outRight + kS);
}

void WristSubsystem::refreshSafeDisconnect()
{
    motorLeft.setDesiredOutput(0);
    motorRight.setDesiredOutput(0);
}
}  // namespace engineer
}  // namespace aruwsrc