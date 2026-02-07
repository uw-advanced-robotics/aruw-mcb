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

#include "aruwsrc/robot/engineer/engineer_control_operator_interface.hpp"

#include "tap/algorithms/math_user_utils.hpp"

#include "aruwsrc/control/chassis/holonomic_chassis_subsystem.hpp"
#include "aruwsrc/robot/engineer/engineer_wrist_constants.hpp"

using namespace tap::algorithms;
using namespace aruwsrc::control::chassis;

namespace aruwsrc::engineer
{
bool EngineerControlOperatorInterface::isDriveMode()
{
    return drivers->remote.getSwitch(Remote::Switch::LEFT_SWITCH) == Remote::SwitchState::DOWN;
}

bool EngineerControlOperatorInterface::isGantryWristControlMode()
{
    return drivers->remote.getSwitch(Remote::Switch::LEFT_SWITCH) == Remote::SwitchState::MID;
}

float EngineerControlOperatorInterface::getCubeLiftVelocity()
{
    // Note: CubeLiftSwitchCommand will break if cubelift is manually moved to the wrong place
    return 0.0f;
}

float EngineerControlOperatorInterface::getGantryExtensionVelocity()
{
    if (getShiftKey())
    {
        if (isGantryWristControlMode())
        {
            return drivers->remote.getMouseX() / divideGantryExtension +
                   drivers->remote.getChannel(Remote::Channel::LEFT_HORIZONTAL);
        }
        else
        {
            return drivers->remote.getMouseX() / divideGantryExtension;
        }
    }
    else
    {
        if (isGantryWristControlMode())
        {
            return drivers->remote.getChannel(Remote::Channel::LEFT_HORIZONTAL);
        }
        return 0.0f;
    }
}

bool EngineerControlOperatorInterface::getGantryKeyIn()
{
    return drivers->remote.keyPressed(Remote::Key::B);
}

bool EngineerControlOperatorInterface::getGantryKeyOut()
{
    return drivers->remote.keyPressed(Remote::Key::G);
}

bool EngineerControlOperatorInterface::getShiftKey()
{
    return drivers->remote.keyPressed(Remote::Key::SHIFT);
}

float EngineerControlOperatorInterface::getWristPitchVelocity()
{
    if (!getShiftKey())
    {
        if (isGantryWristControlMode())
        {
            return -drivers->remote.getChannel(Remote::Channel::RIGHT_VERTICAL) +
                   (drivers->remote.getMouseY() / divideValPitch);
        }
        else
        {
            return drivers->remote.getMouseY() / divideValPitch;
        }
    }
    return 0.0f;
}

float EngineerControlOperatorInterface::getWristYawVelocity()
{
    if (!getShiftKey())
    {
        if (isGantryWristControlMode())
        {
            return -drivers->remote.getChannel(Remote::Channel::RIGHT_HORIZONTAL) -
                   (drivers->remote.getMouseX() / divideValYaw);
        }
        else
        {
            return -drivers->remote.getMouseX() / divideValYaw;
        }
    }
    return 0.0f;
}

float EngineerControlOperatorInterface::getWristRollVelocity()
{
    if (drivers->remote.getMouseL())
    {
        return -aruwsrc::engineer::WRIST_ROLL_CLICK_VELOCITY;
    }
    else if (drivers->remote.getMouseR())
    {
        return aruwsrc::engineer::WRIST_ROLL_CLICK_VELOCITY;
    }
    else if (isGantryWristControlMode())
    {
        return -drivers->remote.getChannel(Remote::Channel::WHEEL);
    }
    else
    {
        return 0.0f;
    }
}

// this is basically the same thing as getChassisXInput in the basic control operator interface, but
// uh... we wanted sprint to work so hopefully the two don't get too out of sync
float EngineerControlOperatorInterface::getChassisXInput()
{
    uint32_t updateCounter = drivers->remote.getUpdateCounter();
    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t dt = currTime - prevChassisXInputCalledTime;
    prevChassisXInputCalledTime = currTime;

    if (prevUpdateCounterX != updateCounter)
    {
        if (isDriveMode())
        {
            chassisXInput.update(
                drivers->remote.getChannel(Remote::Channel::LEFT_VERTICAL),
                currTime);
        }
        else
        {
            chassisXInput.update(0, currTime);
        }
        prevUpdateCounterX = updateCounter;
    }

    float keyInput =
        drivers->remote.keyPressed(Remote::Key::W) - drivers->remote.keyPressed(Remote::Key::S);

    const float maxChassisSpeed =
        aruwsrc::control::chassis::HolonomicChassisSubsystem::getMaxWheelSpeed(
            drivers->refSerial.getRefSerialReceivingData(),
            aruwsrc::control::chassis::HolonomicChassisSubsystem::getChassisPowerLimit(drivers));

    float finalX = maxChassisSpeed *
                   limitVal(chassisXInput.getInterpolatedValue(currTime) + keyInput, -1.0f, 1.0f);

    chassisXInputRamp.setTarget(applyChassisSpeedScaling(finalX));

    ControlOperatorInterface::applyAccelerationToRamp(
        chassisXInputRamp,
        MAX_ACCELERATION_X,
        MAX_DECELERATION_X,
        static_cast<float>(dt) / 1E3F);

    float xInput = chassisXInputRamp.getValue();

    if (drivers->remote.keyPressed(Remote::Key::R))
    {
        return xInput / CHASSIS_SPEED_DIVSOR_SPRINT;
    }
    else
    {
        return xInput / CHASSIS_SPEED_DIVSOR_NORMAL;
    }

    return 0;
}

float EngineerControlOperatorInterface::getChassisYInput()
{
    uint32_t updateCounter = drivers->remote.getUpdateCounter();
    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t dt = currTime - prevChassisYInputCalledTime;
    prevChassisYInputCalledTime = currTime;

    if (prevUpdateCounterY != updateCounter)
    {
        if (isDriveMode())
        {
            chassisYInput.update(
                -drivers->remote.getChannel(Remote::Channel::LEFT_HORIZONTAL),
                currTime);
        }
        else
        {
            chassisYInput.update(0, currTime);
        }

        prevUpdateCounterY = updateCounter;
    }

    float keyInput =
        drivers->remote.keyPressed(Remote::Key::A) - drivers->remote.keyPressed(Remote::Key::D);

    const float maxChassisSpeed =
        aruwsrc::control::chassis::HolonomicChassisSubsystem::getMaxWheelSpeed(
            drivers->refSerial.getRefSerialReceivingData(),
            aruwsrc::control::chassis::HolonomicChassisSubsystem::getChassisPowerLimit(drivers));

    float finalY = maxChassisSpeed *
                   limitVal(chassisYInput.getInterpolatedValue(currTime) + keyInput, -1.0f, 1.0f);

    chassisYInputRamp.setTarget(applyChassisSpeedScaling(finalY));

    applyAccelerationToRamp(
        chassisYInputRamp,
        MAX_ACCELERATION_Y,
        MAX_DECELERATION_Y,
        static_cast<float>(dt) / 1E3F);

    float yInput = chassisYInputRamp.getValue();
    if (drivers->remote.keyPressed(Remote::Key::R))
    {
        return yInput / CHASSIS_SPEED_DIVSOR_SPRINT;
    }
    else
    {
        return yInput / CHASSIS_SPEED_DIVSOR_NORMAL;
    }
}

float EngineerControlOperatorInterface::getChassisRInput()
{
    uint32_t updateCounter = drivers->remote.getUpdateCounter();
    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t dt = currTime - prevChassisRInputCalledTime;
    prevChassisRInputCalledTime = currTime;

    if (prevUpdateCounterR != updateCounter)
    {
        if (isDriveMode())
        {
            chassisRInput.update(
                -drivers->remote.getChannel(Remote::Channel::RIGHT_HORIZONTAL),
                currTime);
        }
        else
        {
            chassisRInput.update(0, currTime);
        }
        prevUpdateCounterR = updateCounter;
    }

    float keyInput =
        drivers->remote.keyPressed(Remote::Key::Q) - drivers->remote.keyPressed(Remote::Key::E);

    const float maxChassisSpeed =
        aruwsrc::control::chassis::HolonomicChassisSubsystem::getMaxWheelSpeed(
            drivers->refSerial.getRefSerialReceivingData(),
            aruwsrc::control::chassis::HolonomicChassisSubsystem::getChassisPowerLimit(drivers));

    float finalR = maxChassisSpeed *
                   limitVal(chassisRInput.getInterpolatedValue(currTime) + keyInput, -1.0f, 1.0f);

    chassisRInputRamp.setTarget(finalR);

    applyAccelerationToRamp(
        chassisRInputRamp,
        MAX_ACCELERATION_R,
        MAX_DECELERATION_R,
        static_cast<float>(dt) / 1E3);

    float rInput = chassisRInputRamp.getValue();
    if (drivers->remote.keyPressed(Remote::Key::R))
    {
        return rInput / CHASSIS_SPEED_DIVSOR_SPRINT;
    }
    else
    {
        return rInput / CHASSIS_SPEED_DIVSOR_NORMAL;
    }
}

}  // namespace aruwsrc::engineer
