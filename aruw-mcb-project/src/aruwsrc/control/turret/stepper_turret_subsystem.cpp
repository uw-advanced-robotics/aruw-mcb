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

#include "stepper_turret_subsystem.hpp"
#include "aruwsrc/drivers.hpp"
#include "tap/errors/create_errors.hpp"

using namespace tap::motor;
using namespace tap::algorithms;

namespace aruwsrc::control::turret
{
StepperTurretSubsystem::StepperTurretSubsystem(
    aruwsrc::Drivers *drivers,
    StepperMotorInterface &pitchMotor,
    StepperMotorInterface &yawMotor)
    : tap::control::Subsystem(drivers), 
      pitchMotor(pitchMotor),
      yawMotor(yawMotor),
      drivers(drivers)
{
    assert(drivers != nullptr);
}

void StepperTurretSubsystem::initialize()
{
    yawMotor.calibrateOrigin(0);
    pitchMotor.calibrateOrigin(0);
}

void StepperTurretSubsystem::refresh()
{
    yawMotor.refresh(); 
    pitchMotor.refresh();
}

void StepperTurretSubsystem::onHardwareTestStart() {
    yawMotor.setDesiredPosition(0);
    pitchMotor.setDesiredPosition(0);
}

}  // namespace aruwsrc::control::turret
