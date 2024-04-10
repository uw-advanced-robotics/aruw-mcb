/*
 * Copyright (c) 2021-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "sentry_turret_subsystem.hpp"

#include "aruwsrc/communication/can/turret_mcb_can_comm.hpp"
#include "aruwsrc/control/turret/constants/turret_constants.hpp"

// @todo: this class should be deprecated
namespace aruwsrc::control::turret
{
SentryTurretSubsystem::SentryTurretSubsystem(
    tap::Drivers* drivers,
    tap::motor::MotorInterface* pitchMotor,
    tap::motor::MotorInterface* yawMotor,
    const TurretMotorConfig& pitchMotorConfig,
    const TurretMotorConfig& yawMotorConfig,
    const aruwsrc::can::TurretMCBCanComm* turretMCB,
    uint8_t turretID)
    : RobotTurretSubsystem(
          drivers,
          pitchMotor,
          yawMotor,
          pitchMotorConfig,
          yawMotorConfig,
          turretMCB),
      turretID(turretID)
{
}

float SentryTurretSubsystem::getWorldYaw() const
{
    return yawMotor.getChassisFrameMeasuredAngle().getValue();
}

float SentryTurretSubsystem::getWorldPitch() const
{
    return pitchMotor.getChassisFrameMeasuredAngle().getValue();
}

uint32_t SentryTurretSubsystem::getLastMeasurementTimeMicros() const
{
    return getTurretMCB()->getIMUDataTimestamp();
}

modm::Vector3f SentryTurretSubsystem::getTurretOffset() const
{
#ifdef TARGET_SENTRY_BEEHIVE
    if (turretID == 1)
    {
        return modm::Vector3f(0, 0, 0);
    }
    else
    {
        return control::turret::OFFSET_TURRET_0_TO_TURRET_1;
    }
#else
    return modm::Vector3f(0, 0, 0);
#endif
}

float SentryTurretSubsystem::getPitchOffset() const
{
#ifdef TARGET_SENTRY_BEEHIVE
    return control::turret::PITCH_YAW_OFFSET;
#else
    return 0;
#endif
}
}  // namespace aruwsrc::control::turret
