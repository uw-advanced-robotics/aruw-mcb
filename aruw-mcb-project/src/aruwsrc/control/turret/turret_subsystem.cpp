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

#include "turret_subsystem.hpp"

#include <algorithm>
#include <cfloat>
#include <random>

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/architecture/clock.hpp"
#include "tap/control/command_mapper.hpp"
#include "tap/errors/create_errors.hpp"

using namespace tap::motor;
using namespace tap::algorithms;

namespace aruwsrc::control::turret
{
TurretSubsystem::TurretSubsystem(
    tap::Drivers *drivers,
    MotorInterface *pitchMotor,
    MotorInterface *yawMotor,
    const TurretMotorConfig &pitchMotorConfig,
    const TurretMotorConfig &yawMotorConfig,
    const aruwsrc::can::TurretMCBCanComm *turretMCB)
    : tap::control::Subsystem(drivers),
      pitchMotor(pitchMotor, pitchMotorConfig),
      yawMotor(yawMotor, yawMotorConfig),
      turretMCB(turretMCB)
{
    assert(drivers != nullptr);
    assert(pitchMotor != nullptr);
    assert(yawMotor != nullptr);
}

void TurretSubsystem::initialize()
{
    yawMotor.initialize();
    pitchMotor.initialize();
}

void TurretSubsystem::refresh()
{
    yawMotor.updateMotorAngle();
    pitchMotor.updateMotorAngle();
}

// void TurretSubsystem::onHardwareTestStart()
// {
//     yawMotor.setMotorOutput(0);
//     pitchMotor.setMotorOutput(0);
// }

}  // namespace aruwsrc::control::turret
