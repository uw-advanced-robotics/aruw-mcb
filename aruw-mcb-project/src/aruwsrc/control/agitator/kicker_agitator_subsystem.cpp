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

#include "kicker_agitator_subsystem.hpp"

#include "aruwsrc/drivers.hpp"

KickerAgitatorSubsystem::KickerAgitatorSubsystem(
    aruwsrc::Drivers *drivers,
    float kp,
    float ki,
    float kd,
    float maxIAccum,
    float maxOutput,
    float agitatorGearRatio,
    tap::motor::MotorId agitatorMotorId,
    tap::can::CanBus agitatorCanBusId,
    bool isAgitatorInverted)
    : tap::control::Subsystem(drivers),
      AgitatorSubsystem(
          drivers,
          kp,
          ki,
          kd,
          maxIAccum,
          maxOutput,
          agitatorGearRatio,
          agitatorMotorId,
          agitatorCanBusId,
          isAgitatorInverted)
{
}

void KickerAgitatorSubsystem::refresh()
{
    updateProjectileQueued();

    if (rotateKicker && launchedProjectile())
    {
        rotateKicker = false;
    }

    if (rotateKicker)
    {
        updateKickerSetpoint();
    }

    AgitatorSubsystem::refresh();
}

void KickerAgitatorSubsystem::launchOneProjectile()
{
    if (projectileQueued)
    {
        rotateKicker = true;
        startRotateSetpoint = getSetpoint();
    }
}

void KickerAgitatorSubsystem::updateProjectileQueued() { projectileQueued = true; }

bool KickerAgitatorSubsystem::launchedProjectile()
{
    if (drivers->refSerial.getRefSerialReceivingData())
    {
        const auto &turret = drivers->refSerial.getRobotData().turret;
        bool ret =
            static_cast<int32_t>(turret.heat42) - prevHeat42 < -HEAT_LIMIT_DIFF_PROJECTILE_LAUNCHED;
        prevHeat42 = turret.heat42;
        return ret;
    }
    else
    {
        return (getSetpoint() - startRotateSetpoint) > ROTATE_SETPOINT_IF_NO_REF_SERIAL;
    }
}

void KickerAgitatorSubsystem::updateKickerSetpoint()
{
    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t dt = currTime - prevTime;
    prevTime = currTime;
    float setpointChange = KICKER_ROTATE_SPEED_RAD_PER_S * dt / 1000.0f;
    setSetpoint(getCurrentValue() + setpointChange);
}
