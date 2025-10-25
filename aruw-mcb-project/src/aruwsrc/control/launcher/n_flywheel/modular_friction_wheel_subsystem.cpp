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

#include "modular_friction_wheel_subsystem.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/architecture/clock.hpp"
#include "tap/drivers.hpp"

#include "aruwsrc/communication/can/turret_mcb_can_comm.hpp"

using namespace tap::algorithms;

namespace aruwsrc::control::launcher::modular_flywheel
{
template<std::size_t NUM_WHEELS>
ModularFrictionWheelSubsystem<NUM_WHEELS>::ModularFrictionWheelSubsystem(
    tap::Drivers *drivers,
    std::array<uint32_t, NUM_WHEELS> wheelIDs,
    std::array<FlywheelConfig, NUM_WHEELS> wheelConfigs,
    tap::can::CanBus canBus,
    aruwsrc::can::TurretMCBCanComm *turretMCB)
    : tap::control::Subsystem(drivers),
      drivers(drivers),
      launchSpeedLinearInterpolator(
          LAUNCH_SPEED_TO_FRICTION_WHEEL_RPM_LUT,
          MODM_ARRAY_SIZE(LAUNCH_SPEED_TO_FRICTION_WHEEL_RPM_LUT)),
      speedCorrectionPid(
          LAUNCHER_SPEED_CORRECTION_PID_KP,
          LAUNCHER_SPEED_CORRECTION_PID_KI,
          LAUNCHER_SPEED_CORRECTION_PID_KD,
          LAUNCHER_SPEED_CORRECTION_PID_MAX_ERROR_SUM,
          LAUNCHER_SPEED_CORRECTION_PID_MAX_OUTPUT),
      desiredRpmRamp(0),
      turretMCB(turretMCB),
      frictionTestCommand(this)
{
    this->setTestCommand(&frictionTestCommand);
    for (int i = 0; i < NUM_WHEELS; i++) {
        wheels[i](drivers, wheelIDs[i], canBus, wheelConfigs[i].isInverted, wheelConfigs[i].name);
    }
}

template<std::size_t NUM_WHEELS>
void ModularFrictionWheelSubsystem<NUM_WHEELS>::initialize()
{
    for (tap::motor::DjiMotor wheel : wheels) {
            wheel.initialize();
        }
    prevTime = tap::arch::clock::getTimeMilliseconds();
}

template<std::size_t NUM_WHEELS>
void ModularFrictionWheelSubsystem<NUM_WHEELS>::setDesiredLaunchSpeed(float speed)
{
    desiredLaunchSpeed = limitVal(speed, 0.0f, MAX_DESIRED_LAUNCH_SPEED);
    desiredRpmRamp.setTarget(launchSpeedToFrictionWheelRpm(speed));
    if (turretMCB != nullptr)
    {
        turretMCB->setLaserStatus(!compareFloatClose(desiredLaunchSpeed, 0, 1E-5));
    }
}

template<std::size_t NUM_WHEELS>
float ModularFrictionWheelSubsystem<NUM_WHEELS>::getCurrentIndividualFrictionWheelSpeed(int index) const
{
    return wheels[index].getEncoder()->getVelocity() * 60.0f / M_TWOPI;
}

template<std::size_t NUM_WHEELS>
float ModularFrictionWheelSubsystem<NUM_WHEELS>::getCurrentAverageFrictionWheelSpeed() const
{
    float sum = 0;
    for (int i = 0; i < NUM_WHEELS; i++) {
        sum += wheels[i].getEncoder()->getVelocity() * 60.0f / M_TWOPI;
    }
    return sum / NUM_WHEELS;
}

template<std::size_t NUM_WHEELS>
void ModularFrictionWheelSubsystem<NUM_WHEELS>::refresh()
{
    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    if (currTime == prevTime)
    {
        return;
    }
    desiredRpmRamp.update(FRICTION_WHEEL_RAMP_SPEED * (currTime - prevTime));
#if defined(ALL_STANDARDS)
    if (prevShotTime != drivers->refSerial.getRobotData().turret.lastReceivedLaunchingInfoTimestamp)
    {
        prevShotTime = drivers->refSerial.getRobotData().turret.lastReceivedLaunchingInfoTimestamp;
        speedCorrectionPid.update(
            drivers->refSerial.getRobotData().turret.bulletSpeed - LAUNCHER_SPEED);
    }
    speedCorrection = speedCorrectionPid.getValue();
#endif

    prevTime = currTime;

    for (int i = 0; i < NUM_WHEELS; i++) {
        wheels[i].velocityPID.update(desiredRpmRamp.getValue() - getCurrentIndividualFrictionWheelSpeed(i) - speedCorrection);
        wheels[i].setDesiredOutput(static_cast<int32_t>(wheels[i].velocityPID.getValue()));
    }
}

template<std::size_t NUM_WHEELS>
float ModularFrictionWheelSubsystem<NUM_WHEELS>::launchSpeedToFrictionWheelRpm(float launchSpeed) const
{
    return launchSpeedLinearInterpolator.interpolate(launchSpeed);
}
}  // namespace aruwsrc::control::launcher::::modular_flywheel
