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

#include "aruwsrc/drivers.hpp"
#include "chassis/chassis_subsystem.hpp"

using namespace tap::algorithms;
using namespace tap::communication::serial;

namespace aruwsrc
{
namespace control
{
float ControlOperatorInterface::applyChassisSpeedScaling(float value)
{
    if (drivers->remote.keyPressed(Remote::Key::CTRL))
    {
        value *= CTRL_SCALAR;
    }
    if (drivers->remote.keyPressed(Remote::Key::SHIFT))
    {
        value *= SHIFT_SCALAR;
    }

    return value;
}

static inline void applyChassisAcceleration(
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

float ControlOperatorInterface::getChassisXInput()
{
    uint32_t updateCounter = drivers->remote.getUpdateCounter();
    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t dt = currTime - prevChassisXInuptCalledTime;
    prevChassisXInuptCalledTime = currTime;

    if (prevUpdateCounterX != updateCounter)
    {
        chassisXInput.update(drivers->remote.getChannel(Remote::Channel::LEFT_VERTICAL), currTime);
        prevUpdateCounterX = updateCounter;
    }

    float keyInput =
        drivers->remote.keyPressed(Remote::Key::W) - drivers->remote.keyPressed(Remote::Key::S);

    const float MAX_CHASSIS_SPEED = chassis::ChassisSubsystem::getMaxUserWheelSpeed(
        drivers->refSerial.getRefSerialReceivingData(),
        drivers->refSerial.getRobotData().chassis.power);

    float finalX = MAX_CHASSIS_SPEED *
                   limitVal(chassisXInput.getInterpolatedValue(currTime) + keyInput, -1.0f, 1.0f);

    chassisXInputRamp.setTarget(applyChassisSpeedScaling(finalX));

    applyChassisAcceleration(
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
    uint32_t dt = currTime - prevChassisYInuptCalledTime;
    prevChassisYInuptCalledTime = currTime;

    if (prevUpdateCounterY != updateCounter)
    {
        chassisYInput.update(
            -drivers->remote.getChannel(Remote::Channel::LEFT_HORIZONTAL),
            currTime);
        prevUpdateCounterY = updateCounter;
    }

    float keyInput =
        drivers->remote.keyPressed(Remote::Key::A) - drivers->remote.keyPressed(Remote::Key::D);

    const float MAX_CHASSIS_SPEED = chassis::ChassisSubsystem::getMaxUserWheelSpeed(
        drivers->refSerial.getRefSerialReceivingData(),
        drivers->refSerial.getRobotData().chassis.power);

    float finalY = MAX_CHASSIS_SPEED *
                   limitVal(chassisYInput.getInterpolatedValue(currTime) + keyInput, -1.0f, 1.0f);

    chassisYInputRamp.setTarget(applyChassisSpeedScaling(finalY));

    applyChassisAcceleration(
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
    uint32_t dt = currTime - prevChassisRInuptCalledTime;
    prevChassisRInuptCalledTime = currTime;

    if (prevUpdateCounterR != updateCounter)
    {
        chassisRInput.update(
            -drivers->remote.getChannel(Remote::Channel::RIGHT_HORIZONTAL),
            currTime);
        prevUpdateCounterR = updateCounter;
    }

    float keyInput =
        drivers->remote.keyPressed(Remote::Key::Q) - drivers->remote.keyPressed(Remote::Key::E);

    const float MAX_CHASSIS_SPEED = chassis::ChassisSubsystem::getMaxUserWheelSpeed(
        drivers->refSerial.getRefSerialReceivingData(),
        drivers->refSerial.getRobotData().chassis.power);

    float finalR = MAX_CHASSIS_SPEED *
                   limitVal(chassisRInput.getInterpolatedValue(currTime) + keyInput, -1.0f, 1.0f);

    chassisRInputRamp.setTarget(finalR);

    applyChassisAcceleration(
        chassisRInputRamp,
        MAX_ACCELERATION_R,
        MAX_DECELERATION_R,
        static_cast<float>(dt) / 1E3);

    return chassisRInputRamp.getValue();
}

float ControlOperatorInterface::getTurretYawInput()
{
    return -drivers->remote.getChannel(Remote::Channel::RIGHT_HORIZONTAL) +
           static_cast<float>(limitVal<int16_t>(
               -drivers->remote.getMouseX(),
               -USER_MOUSE_YAW_MAX,
               USER_MOUSE_YAW_MAX)) *
               USER_MOUSE_YAW_SCALAR;
}

float ControlOperatorInterface::getTurretPitchInput()
{
    return -drivers->remote.getChannel(Remote::Channel::RIGHT_VERTICAL) +
           static_cast<float>(limitVal<int16_t>(
               drivers->remote.getMouseY(),
               -USER_MOUSE_PITCH_MAX,
               USER_MOUSE_PITCH_MAX)) *
               USER_MOUSE_PITCH_SCALAR;
}

float ControlOperatorInterface::getSentinelSpeedInput()
{
    return drivers->remote.getChannel(Remote::Channel::LEFT_HORIZONTAL) *
           USER_STICK_SENTINEL_DRIVE_SCALAR;
}
}  // namespace control

}  // namespace aruwsrc
