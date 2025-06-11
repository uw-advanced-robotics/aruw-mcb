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

#include "engineer_control_operator_interface.hpp"

#include "tap/algorithms/math_user_utils.hpp"

#include "aruwsrc/control/chassis/holonomic_chassis_subsystem.hpp"

using namespace tap::algorithms;
using namespace tap::communication::serial;

namespace aruwsrc::control::engineer
{
bool EngineerControlOperatorInterface::isDriveMode()
{
    return drivers->remote.getSwitch(Remote::Switch::LEFT_SWITCH) == Remote::SwitchState::DOWN &&
           drivers->remote.getSwitch(Remote::Switch::RIGHT_SWITCH) != Remote::SwitchState::UP;
}

bool EngineerControlOperatorInterface::isGantryControlMode()
{
    return drivers->remote.getSwitch(Remote::Switch::LEFT_SWITCH) == Remote::SwitchState::MID &&
           drivers->remote.getSwitch(Remote::Switch::RIGHT_SWITCH) != Remote::SwitchState::UP;
}

bool EngineerControlOperatorInterface::isWristControlMode()
{
    return drivers->remote.getSwitch(Remote::Switch::LEFT_SWITCH) == Remote::SwitchState::UP &&
           drivers->remote.getSwitch(Remote::Switch::RIGHT_SWITCH) != Remote::SwitchState::UP;
}

float EngineerControlOperatorInterface::getCubeLiftVelocity()
{
    if (isGantryControlMode())
        return drivers->remote.getChannel(Remote::Channel::WHEEL);
    else
        return 0.0f;
}

float EngineerControlOperatorInterface::getGantryLiftVelocity()
{
    if (isGantryControlMode())
        return drivers->remote.getChannel(Remote::Channel::RIGHT_VERTICAL);
    else
        return 0.0f;
}

float EngineerControlOperatorInterface::getGantryExtensionVelocity()
{
    if (isGantryControlMode())
        return drivers->remote.getChannel(Remote::Channel::LEFT_HORIZONTAL);
    else
        return 0.0f;
}

float EngineerControlOperatorInterface::getWristPitchVelocity()
{
    if (isWristControlMode())
        return drivers->remote.getChannel(Remote::Channel::RIGHT_VERTICAL);
    else
        return 0.0f;
}

float EngineerControlOperatorInterface::getWristYawVelocity()
{
    if (isWristControlMode())
        return drivers->remote.getChannel(Remote::Channel::LEFT_HORIZONTAL);
    else
        return 0.0f;
}

float EngineerControlOperatorInterface::getWristRollVelocity()
{
    if (isWristControlMode())
        return drivers->remote.getChannel(Remote::Channel::WHEEL);
    else
        return 0.0f;
}

float EngineerControlOperatorInterface::getChassisXInput()
{
    uint32_t updateCounter = drivers->remote.getUpdateCounter();
    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t dt = currTime - prevChassisXInputCalledTime;
    prevChassisXInputCalledTime = currTime;

    if (prevUpdateCounterX != updateCounter)
    {
        chassisXInput.update(
            drivers->remote.getChannel(Remote::Channel::LEFT_VERTICAL) * isDriveMode(),
            currTime);
        prevUpdateCounterX = updateCounter;
    }

    float keyInput =
        drivers->remote.keyPressed(Remote::Key::W) - drivers->remote.keyPressed(Remote::Key::S);

    const float maxChassisSpeed = chassis::HolonomicChassisSubsystem::getMaxWheelSpeed(
        drivers->refSerial.getRefSerialReceivingData(),
        chassis::HolonomicChassisSubsystem::getChassisPowerLimit(drivers));

    float finalX = maxChassisSpeed *
                   limitVal(chassisXInput.getInterpolatedValue(currTime) + keyInput, -1.0f, 1.0f);

    chassisXInputRamp.setTarget(applyChassisSpeedScaling(finalX));

    applyAccelerationToRamp(
        chassisXInputRamp,
        MAX_ACCELERATION_X,
        MAX_DECELERATION_X,
        static_cast<float>(dt) / 1E3F);

    return chassisXInputRamp.getValue();
}

float EngineerControlOperatorInterface::getChassisYInput()
{
    uint32_t updateCounter = drivers->remote.getUpdateCounter();
    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t dt = currTime - prevChassisYInputCalledTime;
    prevChassisYInputCalledTime = currTime;

    if (prevUpdateCounterY != updateCounter)
    {
        chassisYInput.update(
            -drivers->remote.getChannel(Remote::Channel::LEFT_HORIZONTAL) * isDriveMode(),
            currTime);
        prevUpdateCounterY = updateCounter;
    }

    float keyInput =
        drivers->remote.keyPressed(Remote::Key::A) - drivers->remote.keyPressed(Remote::Key::D);

    const float maxChassisSpeed = chassis::HolonomicChassisSubsystem::getMaxWheelSpeed(
        drivers->refSerial.getRefSerialReceivingData(),
        chassis::HolonomicChassisSubsystem::getChassisPowerLimit(drivers));

    float finalY = maxChassisSpeed *
                   limitVal(chassisYInput.getInterpolatedValue(currTime) + keyInput, -1.0f, 1.0f);

    chassisYInputRamp.setTarget(applyChassisSpeedScaling(finalY));

    applyAccelerationToRamp(
        chassisYInputRamp,
        MAX_ACCELERATION_Y,
        MAX_DECELERATION_Y,
        static_cast<float>(dt) / 1E3F);

    return chassisYInputRamp.getValue();
}

float EngineerControlOperatorInterface::getChassisRInput()
{
    uint32_t updateCounter = drivers->remote.getUpdateCounter();
    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t dt = currTime - prevChassisRInputCalledTime;
    prevChassisRInputCalledTime = currTime;

    if (prevUpdateCounterR != updateCounter)
    {
        chassisRInput.update(
            -drivers->remote.getChannel(Remote::Channel::RIGHT_HORIZONTAL) * isDriveMode(),
            currTime);
        prevUpdateCounterR = updateCounter;
    }

    float keyInput =
        drivers->remote.keyPressed(Remote::Key::Q) - drivers->remote.keyPressed(Remote::Key::E);

    const float maxChassisSpeed = chassis::HolonomicChassisSubsystem::getMaxWheelSpeed(
        drivers->refSerial.getRefSerialReceivingData(),
        chassis::HolonomicChassisSubsystem::getChassisPowerLimit(drivers));

    float finalR = maxChassisSpeed *
                   limitVal(chassisRInput.getInterpolatedValue(currTime) + keyInput, -1.0f, 1.0f);

    chassisRInputRamp.setTarget(finalR);

    applyAccelerationToRamp(
        chassisRInputRamp,
        MAX_ACCELERATION_R,
        MAX_DECELERATION_R,
        static_cast<float>(dt) / 1E3);

    return chassisRInputRamp.getValue();
}

}  // namespace aruwsrc::control::engineer
