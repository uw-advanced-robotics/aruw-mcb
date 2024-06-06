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

#include "aruwsrc/robot/sentry/sentry_control_operator_interface.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/architecture/clock.hpp"

#include "aruwsrc/control/chassis/holonomic_chassis_subsystem.hpp"

using namespace tap::algorithms;
using namespace tap::communication::serial;

namespace aruwsrc
{
namespace control::sentry
{
bool SentryControlOperatorInterface::isTurretControlMode()
{
    Remote::SwitchState leftState = drivers->remote.getSwitch(Remote::Switch::LEFT_SWITCH);
    Remote::SwitchState rightState = drivers->remote.getSwitch(Remote::Switch::RIGHT_SWITCH);

    return (leftState == Remote::SwitchState::MID && rightState == Remote::SwitchState::UP) ||
           (leftState == Remote::SwitchState::MID && rightState == Remote::SwitchState::MID) ||
           (leftState == Remote::SwitchState::MID && rightState == Remote::SwitchState::DOWN);
}

bool SentryControlOperatorInterface::isDriveMode()
{
    // return drivers->remote.getSwitch(Remote::Switch::RIGHT_SWITCH) == Remote::SwitchState::MID);
    Remote::SwitchState leftState = drivers->remote.getSwitch(Remote::Switch::LEFT_SWITCH);
    Remote::SwitchState rightState = drivers->remote.getSwitch(Remote::Switch::RIGHT_SWITCH);

    return (leftState == Remote::SwitchState::DOWN && rightState == Remote::SwitchState::UP) ||
           (leftState == Remote::SwitchState::DOWN && rightState == Remote::SwitchState::MID) ||
           (leftState == Remote::SwitchState::DOWN && rightState == Remote::SwitchState::DOWN);
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

float SentryControlOperatorInterface::getChassisXVelocity()
{
    if (!isDriveMode()) return DEFAULT_CHASSIS_X_VELOCITY;

    // Set dt and previous time
    uint32_t updateCounter = drivers->remote.getUpdateCounter();
    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t dt = currTime - prevChassisXInputCalledTime;
    prevChassisXInputCalledTime = currTime;

    if (prevUpdateCounterChassisXInput != updateCounter)
    {
        // coord system shift (-)
        chassisXInput.update(drivers->remote.getChannel(Remote::Channel::LEFT_VERTICAL), currTime);
        prevUpdateCounterChassisXInput = updateCounter;
    }

    const float maxChassisSpeed = chassis::HolonomicChassisSubsystem::getMaxWheelSpeed(
        drivers->refSerial.getRefSerialReceivingData(),
        drivers->refSerial.getRobotData().chassis.power);

    float finalX =
        maxChassisSpeed * limitVal(chassisXInput.getInterpolatedValue(currTime), -1.0f, 1.0f);

    chassisXInputRamp.setTarget(finalX);

    applyAccelerationToRamp(
        chassisXInputRamp,
        MAX_ACCELERATION_X,
        MAX_DECELERATION_X,
        static_cast<float>(dt) / 1E3F);

    return chassisXInputRamp.getValue();
}

float SentryControlOperatorInterface::getChassisYVelocity()
{
    if (!isDriveMode()) return DEFAULT_CHASSIS_Y_VELOCITY;

    uint32_t updateCounter = drivers->remote.getUpdateCounter();
    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t dt = currTime - prevChassisYInputCalledTime;
    prevUpdateCounterChassisYInput = currTime;

    if (prevUpdateCounterChassisYInput != updateCounter)
    {
        chassisYInput.update(
            -drivers->remote.getChannel(Remote::Channel::LEFT_HORIZONTAL),
            currTime);
        prevUpdateCounterChassisYInput = updateCounter;
    }

    const float maxChassisSpeed = chassis::HolonomicChassisSubsystem::getMaxWheelSpeed(
        drivers->refSerial.getRefSerialReceivingData(),
        drivers->refSerial.getRobotData().chassis.power);

    float finalY =
        maxChassisSpeed * limitVal(chassisYInput.getInterpolatedValue(currTime), -1.0f, 1.0f);

    chassisYInputRamp.setTarget(finalY);

    applyAccelerationToRamp(
        chassisYInputRamp,
        MAX_ACCELERATION_Y,
        MAX_DECELERATION_Y,
        static_cast<float>(dt) / 1E3F);
    return chassisYInputRamp.getValue();
}

float SentryControlOperatorInterface::getChassisYawVelocity()
{
    if (!isDriveMode()) return DEFAULT_CHASSIS_YAW_VELOCITY;

    uint32_t updateCounter = drivers->remote.getUpdateCounter();
    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t dt = currTime - prevChassisYawnputCalledTime;  // @todo typo lol
    prevChassisYawnputCalledTime = currTime;

    if (prevUpdateCounterChassisYawInput != updateCounter)
    {
        chassisYawInput.update(
            -drivers->remote.getChannel(Remote::Channel::RIGHT_HORIZONTAL),
            currTime);
        prevUpdateCounterChassisYawInput = updateCounter;
    }

    const float maxChassisYawSpeed = MAX_CHASSIS_YAW_SPEED;

    float finalR =
        maxChassisYawSpeed * limitVal(chassisYawInput.getInterpolatedValue(currTime), -1.0f, 1.0f);

    chassisYawInputRamp.setTarget(finalR);

    applyAccelerationToRamp(
        chassisYawInputRamp,
        MAX_ACCELERATION_R,
        MAX_DECELERATION_R,
        static_cast<float>(dt) / 1E3);

    return chassisYawInputRamp.getValue() * 20;
}

float SentryControlOperatorInterface::getTurretMajorYawVelocity()
{
    if (!isTurretControlMode()) return DEFAULT_TURRET_MAJOR_VELOCITY;

    return drivers->remote.getChannel(Remote::Channel::WHEEL);
}

float SentryControlOperatorInterface::getTurretMinor1YawVelocity()
{
    if (!isTurretControlMode()) return 0.f;

    return -drivers->remote.getChannel(Remote::Channel::LEFT_HORIZONTAL);
}

float SentryControlOperatorInterface::getTurretMinor1PitchVelocity()
{
    if (!isTurretControlMode()) return 0.f;

    return -drivers->remote.getChannel(Remote::Channel::LEFT_VERTICAL);
}

float SentryControlOperatorInterface::getTurretMinor2YawVelocity()
{
    if (!isTurretControlMode()) return 0.f;

    return -drivers->remote.getChannel(Remote::Channel::RIGHT_HORIZONTAL);
}

float SentryControlOperatorInterface::getTurretMinor2PitchVelocity()
{
    if (!isTurretControlMode()) return 0.f;

    return -drivers->remote.getChannel(Remote::Channel::RIGHT_VERTICAL);
}

}  // namespace control::sentry
}  // namespace aruwsrc
