/*
 * Copyright (c) 2020-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "control_operator_interface.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/architecture/clock.hpp"
#include "tap/drivers.hpp"

#include "aruwsrc/control/chassis/new-chassis/chassis_subsystem.hpp"
#include "aruwsrc/control/turret/constants/turret_constants.hpp"

using namespace tap::algorithms;
using namespace tap::communication::serial;

namespace aruwsrc
{
namespace control
{
float ControlOperatorInterface::applyChassisSpeedScaling(float value)
{
    if (isSlowMode())
    {
        value *= SPEED_REDUCTION_SCALAR;
    }
    return value;
}

bool ControlOperatorInterface::isSlowMode()
{
    return drivers->remote.keyPressed(Remote::Key::CTRL);
}

/**
 * @param[out] ramp Ramp that should have acceleration applied to. The ramp is updated some
 * increment based on the passed in acceleration values. Ramp stores values in some units.
 * @param[in] maxAcceleration Positive acceleration value to apply to the ramp in units/time^2.
 * @param[in] maxDeceleration Negative acceleration value to apply to the ramp, in units/time^2.
 * @param[in] dt Change in time since this function was last called, in units of some time.
 */
static inline void applyAccelerationToRamp(
    tap::algorithms::Ramp &ramp,
    float maxAcceleration,
    float maxDeceleration,
    float dt)
{
    if (getSign(ramp.getTarget()) == getSign(ramp.getValue()) &&
        abs(ramp.getTarget()) > abs(ramp.getValue()))
    {
        // we are trying to speed up
        ramp.update(maxAcceleration * dt);
    }
    else
    {
        // we are trying to slow down
        ramp.update(maxDeceleration * dt);
    }
}

// range of values for getChassisXInput(), getChassisYInput(), getChassisRInput() is 0-500

float ControlOperatorInterface::getChassisXInput()
{
    uint32_t updateCounter = drivers->remote.getUpdateCounter();
    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t dt = currTime - prevChassisXInputCalledTime;
    prevChassisXInputCalledTime = currTime;

    if (prevUpdateCounterX != updateCounter)
    {
        chassisXInput.update(drivers->remote.getChannel(Remote::Channel::LEFT_VERTICAL), currTime);
        prevUpdateCounterX = updateCounter;
    }

    float keyInput =
        drivers->remote.keyPressed(Remote::Key::W) - drivers->remote.keyPressed(Remote::Key::S);

    const float maxChassisSpeed = chassis::ChassisSubsystem::getMaxWheelSpeed(
        drivers->refSerial.getRefSerialReceivingData(),
        drivers->refSerial.getRobotData().chassis.power);

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

float ControlOperatorInterface::getChassisYInput()
{
    uint32_t updateCounter = drivers->remote.getUpdateCounter();
    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t dt = currTime - prevChassisYInputCalledTime;
    prevChassisYInputCalledTime = currTime;

    if (prevUpdateCounterY != updateCounter)
    {
        chassisYInput.update(
            -drivers->remote.getChannel(Remote::Channel::LEFT_HORIZONTAL),
            currTime);
        prevUpdateCounterY = updateCounter;
    }

    float keyInput =
        drivers->remote.keyPressed(Remote::Key::A) - drivers->remote.keyPressed(Remote::Key::D);

    const float maxChassisSpeed = chassis::ChassisSubsystem::getMaxWheelSpeed(
        drivers->refSerial.getRefSerialReceivingData(),
        drivers->refSerial.getRobotData().chassis.power);

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

float ControlOperatorInterface::getChassisRInput()
{
    uint32_t updateCounter = drivers->remote.getUpdateCounter();
    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t dt = currTime - prevChassisRInputCalledTime;
    prevChassisRInputCalledTime = currTime;

    if (prevUpdateCounterR != updateCounter)
    {
        chassisRInput.update(
            -drivers->remote.getChannel(Remote::Channel::RIGHT_HORIZONTAL),
            currTime);
        prevUpdateCounterR = updateCounter;
    }

    float keyInput =
        drivers->remote.keyPressed(Remote::Key::Q) - drivers->remote.keyPressed(Remote::Key::E);

    const float maxChassisSpeed = chassis::ChassisSubsystem::getMaxWheelSpeed(
        drivers->refSerial.getRefSerialReceivingData(),
        drivers->refSerial.getRobotData().chassis.power);  // rpm

    float finalR =
        maxChassisSpeed *
        limitVal(chassisRInput.getInterpolatedValue(currTime) + keyInput, -1.0f, 1.0f);  // rpm

    chassisRInputRamp.setTarget(finalR);

    applyAccelerationToRamp(
        chassisRInputRamp,
        MAX_ACCELERATION_R,
        MAX_DECELERATION_R,
        static_cast<float>(dt) / 1E3);

    return chassisRInputRamp.getValue();
}

float ControlOperatorInterface::getTurretYawInput(uint8_t turretID)
{
    switch (turretID)
    {
        case 0:
            return -drivers->remote.getChannel(Remote::Channel::RIGHT_HORIZONTAL) +
                   static_cast<float>(limitVal<int16_t>(
                       -drivers->remote.getMouseX(),
                       -USER_MOUSE_YAW_MAX,
                       USER_MOUSE_YAW_MAX)) *
                       USER_MOUSE_YAW_SCALAR;
        case 1:
            return -drivers->remote.getChannel(Remote::Channel::LEFT_HORIZONTAL) +
                   static_cast<float>(limitVal<int16_t>(
                       -drivers->remote.getMouseX(),
                       -USER_MOUSE_YAW_MAX,
                       USER_MOUSE_YAW_MAX)) *
                       USER_MOUSE_YAW_SCALAR;
        default:
            return 0;
    }
}

float ControlOperatorInterface::getTurretPitchInput(uint8_t turretID)
{
    switch (turretID)
    {
        case 0:
            return -drivers->remote.getChannel(Remote::Channel::RIGHT_VERTICAL) +
                   static_cast<float>(limitVal<int16_t>(
                       drivers->remote.getMouseY(),
                       -USER_MOUSE_PITCH_MAX,
                       USER_MOUSE_PITCH_MAX)) *
                       USER_MOUSE_PITCH_SCALAR;
        case 1:
            return -drivers->remote.getChannel(Remote::Channel::LEFT_VERTICAL) +
                   static_cast<float>(limitVal<int16_t>(
                       drivers->remote.getMouseY(),
                       -USER_MOUSE_PITCH_MAX,
                       USER_MOUSE_PITCH_MAX)) *
                       USER_MOUSE_PITCH_SCALAR;
        default:
            return 0;
    }
}

float ControlOperatorInterface::getSentrySpeedInput()
{
    return (-drivers->remote.getChannel(Remote::Channel::WHEEL) / 660.0f) *
           USER_STICK_SENTRY_DRIVE_SCALAR;
}
}  // namespace control

}  // namespace aruwsrc
