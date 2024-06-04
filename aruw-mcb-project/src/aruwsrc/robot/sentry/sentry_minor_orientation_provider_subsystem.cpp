/*
 * Copyright (c) 2024-2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "sentry_minor_orientation_provider_subsystem.hpp"

namespace aruwsrc::control::turret
{

SentryMinorOrientationProviderSubsystem::SentryMinorOrientationProviderSubsystem(
    tap::Drivers* drivers,
    const TurretMotor& turretYawMotor,
    const aruwsrc::can::TurretMCBCanComm& turretMCB,
    const transforms::Transform& worldToMajor,
    const tap::algorithms::SmoothPidConfig& driftPidConfig)
    : Subsystem(drivers),
      turretYawMotor(turretYawMotor),
      turretMCB(turretMCB),
      worldToMajor(worldToMajor),
      yawCorrectionPid(driftPidConfig)
{
}

void SentryMinorOrientationProviderSubsystem::initialize()
{
    prevtimeMillis = tap::arch::clock::getTimeMilliseconds();
    zero();
}

void SentryMinorOrientationProviderSubsystem::zero()
{
    yawCorrection = 0;
    yawCorrectionPid.reset();
}

void SentryMinorOrientationProviderSubsystem::refresh()
{
    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t dt = currTime - prevtimeMillis;

    yawCorrection = yawCorrectionPid.runControllerDerivateError(
        getYaw().minDifference(getBaselineYaw()),
        dt / 1000.0f);

    prevtimeMillis = currTime;
}

Angle SentryMinorOrientationProviderSubsystem::getBaselineYaw() const
{
    return worldToMajor.getYaw() + turretYawMotor.getAngleFromCenter();
}

}  // namespace aruwsrc::control::turret