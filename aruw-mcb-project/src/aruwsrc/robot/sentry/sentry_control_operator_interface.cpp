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

#include "sentry_control_operator_interface.hpp"
#include "tap/architecture/clock.hpp"
#include "aruwsrc/control/chassis/holonomic_chassis_subsystem.hpp"
#include "tap/algorithms/math_user_utils.hpp"
#include "aruwsrc/control/turret/constants/turret_constants.hpp"


using namespace tap::algorithms;
using namespace tap::communication::serial;

namespace aruwsrc
{
namespace control::sentry
{

 bool SentryControlOperatorInterface::isAutoDriveMode() 
 {
    return drivers->remote.getSwitch(Remote::Switch::LEFT_SWITCH) == Remote::SwitchState::UP;
 }

bool SentryControlOperatorInterface::isTurretControlMode() 
 {
    return drivers->remote.getSwitch(Remote::Switch::LEFT_SWITCH) == Remote::SwitchState::MID;
 }

bool SentryControlOperatorInterface::isDriveMode() 
 {
    return drivers->remote.getSwitch(Remote::Switch::LEFT_SWITCH) == Remote::SwitchState::DOWN;
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
        chassisXInput.update(drivers->remote.getChannel(Remote::Channel::LEFT_VERTICAL), currTime);
        prevUpdateCounterChassisXInput = updateCounter;
    }

    const float maxChassisSpeed = chassis::HolonomicChassisSubsystem::getMaxWheelSpeed(
        drivers->refSerial.getRefSerialReceivingData(),
        drivers->refSerial.getRobotData().chassis.power);

    float finalX = maxChassisSpeed *
                   limitVal(chassisXInput.getInterpolatedValue(currTime), -1.0f, 1.0f);

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
    // Set dt and previous time
    uint32_t updateCounter = drivers->remote.getUpdateCounter();
    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t dt = currTime - prevChassisYInputCalledTime;
    prevUpdateCounterChassisYInput = currTime;

    if (prevUpdateCounterChassisYInput != updateCounter)
    {
        chassisXInput.update(drivers->remote.getChannel(Remote::Channel::LEFT_HORIZONTAL), currTime);
        prevUpdateCounterChassisYInput = updateCounter;
    }

    const float maxChassisSpeed = chassis::HolonomicChassisSubsystem::getMaxWheelSpeed(
        drivers->refSerial.getRefSerialReceivingData(),
        drivers->refSerial.getRobotData().chassis.power);

    float finalY = maxChassisSpeed *
                   limitVal(chassisYInput.getInterpolatedValue(currTime), -1.0f, 1.0f);

    chassisYInputRamp.setTarget(finalY); 

    applyAccelerationToRamp(
        chassisYInputRamp,
        MAX_ACCELERATION_Y,
        MAX_DECELERATION_Y,
        static_cast<float>(dt) / 1E3F);
    return chassisYInputRamp.getValue();
}

float SentryControlOperatorInterface::getTurretMajorYawVelocity()
{
    if (!isDriveMode()) return DEFAULT_TURRET_MAJOR_VELOCITY;
    
    uint32_t updateCounter = drivers->remote.getUpdateCounter();
    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t dt = currTime - prevTurretMajorYawInputCalledTime;
    prevTurretMajorYawInputCalledTime = currTime;

    if (prevUpdateCounterTurretMajorYawInput != updateCounter)
    {
        turretMajorYawInput.update(
            -drivers->remote.getChannel(Remote::Channel::RIGHT_HORIZONTAL),
            currTime);
        prevUpdateCounterTurretMajorYawInput = updateCounter;
    }
    
    const float maxTurretMajorYawSpeed = MAX_TURRET_MAJOR_YAW_SPEED;

    float finalR = maxTurretMajorYawSpeed *
                   limitVal(turretMajorYawInput.getInterpolatedValue(currTime), -1.0f, 1.0f);

    turretMajorYawRamp.setTarget(finalR);

    applyAccelerationToRamp(
        turretMajorYawRamp,
        MAX_ACCELERATION_R,
        MAX_DECELERATION_R,
        static_cast<float>(dt) / 1E3);

    return turretMajorYawRamp.getValue();
}

float SentryControlOperatorInterface::getTurretMinor1YawVelocity()
{
    if (!isTurretControlMode()) return 0.f;
    
    uint32_t updateCounter = drivers->remote.getUpdateCounter();
    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t dt = currTime - prevTurretMinor1YawInputCalledTime;
    prevTurretMinor1YawInputCalledTime = currTime;

    if (prevUpdateCounterTurretMinor1YawInput != updateCounter)
    {
        turretMinor1YawInput.update(
            -drivers->remote.getChannel(Remote::Channel::RIGHT_HORIZONTAL),
            currTime);
        prevUpdateCounterTurretMinor1YawInput = updateCounter;
    }
    
    const float maxTurretMinor1YawSpeed = MAX_TURRET1_MINOR_YAW_SPEED;

    float finalYaw = maxTurretMinor1YawSpeed *
                   limitVal(turretMinor1YawInput.getInterpolatedValue(currTime), -1.0f, 1.0f);

    turretMinor1YawRamp.setTarget(finalYaw);

    applyAccelerationToRamp(
        turretMinor1YawRamp,
        MAX_ACCELERATION_R,
        MAX_DECELERATION_R,
        static_cast<float>(dt) / 1E3);

    return turretMinor1YawRamp.getValue();
}

float SentryControlOperatorInterface::getTurretMinor1PitchVelocity()
{
    if (!isTurretControlMode()) return 0.f;
    
    uint32_t updateCounter = drivers->remote.getUpdateCounter();
    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t dt = currTime - prevTurretMinor1PitchInputCalledTime;
    prevTurretMinor1PitchInputCalledTime = currTime;

    if (prevUpdateCounterTurretMinor1PitchInput != updateCounter)
    {
        turretMinor1YawInput.update(
            -drivers->remote.getChannel(Remote::Channel::LEFT_VERTICAL),
            currTime);
        prevUpdateCounterTurretMinor1PitchInput = updateCounter;
    }
    
    const float maxTurretMinor1PitchSpeed = MAX_TURRET1_MINOR_PITCH_SPEED;

    float finalPitch = maxTurretMinor1PitchSpeed *
                   limitVal(turretMinor1PitchInput.getInterpolatedValue(currTime), -1.0f, 1.0f);

    turretMinor1PitchRamp.setTarget(finalPitch);

    applyAccelerationToRamp(
        turretMinor1PitchRamp,
        MAX_ACCELERATION_R,
        MAX_DECELERATION_R,
        static_cast<float>(dt) / 1E3);

    return turretMinor1PitchRamp.getValue();
}

float SentryControlOperatorInterface::getTurretMinor2YawVelocity()
{
    if (!isTurretControlMode()) return 0.f;
    
    uint32_t updateCounter = drivers->remote.getUpdateCounter();
    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t dt = currTime - prevTurretMinor2YawInputCalledTime;
    prevTurretMinor2YawInputCalledTime = currTime;

    if (prevUpdateCounterTurretMinor2YawInput != updateCounter)
    {
        turretMinor2YawInput.update(
            -drivers->remote.getChannel(Remote::Channel::RIGHT_HORIZONTAL),
            currTime);
        prevUpdateCounterTurretMinor2YawInput = updateCounter;
    }
    
    const float maxTurretMinor2YawSpeed = MAX_TURRET2_MINOR_YAW_SPEED;

    float finalYaw = maxTurretMinor2YawSpeed *
                   limitVal(turretMinor2YawInput.getInterpolatedValue(currTime), -1.0f, 1.0f);

    turretMinor2YawRamp.setTarget(finalYaw);

    applyAccelerationToRamp(
        turretMinor2YawRamp,
        MAX_ACCELERATION_R,
        MAX_DECELERATION_R,
        static_cast<float>(dt) / 1E3);

    return turretMinor2YawRamp.getValue();
}

float SentryControlOperatorInterface::getTurretMinor2PitchVelocity()
{
    if (!isTurretControlMode()) return 0.f; 

    uint32_t updateCounter = drivers->remote.getUpdateCounter();
    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t dt = currTime - prevTurretMinor2PitchInputCalledTime;
    prevTurretMinor2PitchInputCalledTime = currTime;

    if (prevUpdateCounterTurretMinor2PitchInput != updateCounter)
    {
        turretMinor2PitchInput.update(
            -drivers->remote.getChannel(Remote::Channel::RIGHT_VERTICAL),
            currTime);
        prevUpdateCounterTurretMinor2PitchInput = updateCounter;
    }
    
    const float maxTurretMinor2PitchSpeed = MAX_TURRET2_MINOR_PITCH_SPEED;

    float finalPitch = maxTurretMinor2PitchSpeed *
                   limitVal(turretMinor2PitchRamp.getInterpolatedValue(currTime), -1.0f, 1.0f);

    turretMinor1PitchRamp.setTarget(finalPitch);

    applyAccelerationToRamp(
        turretMinor1PitchRamp,
        MAX_ACCELERATION_R,
        MAX_DECELERATION_R,
        static_cast<float>(dt) / 1E3);

    return turretMinor1PitchRamp.getValue();
}


} // namespace aruwsrc::control::sentry
} // namespace aruwsrc