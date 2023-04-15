/*
 * Copyright (c) 2020-2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

<<<<<<< HEAD:aruw-mcb-project/src/aruwsrc/algorithms/wheel.cpp
#include "wheel.hpp"

namespace aruwsrc
{
class Drivers;
}

namespace aruwsrc
{
namespace algorithms
{
Wheel::Wheel(float diameter, float gearRatio, float motorGearRatio)
    : circumference(diameter * M_PI),
      gearRatio(gearRatio),
      motorGearRatio(motorGearRatio)
{
}

float Wheel::mpsToRpm(float mps) const
{
    return (mps / circumference) / motorGearRatio * 60.0f / gearRatio;
}

float Wheel::rpmToMps(float rpm) const
{
    return rpm * motorGearRatio / 60.0f * gearRatio * circumference;
}

}  // namespace algorithms

}  // namespace aruwsrc
=======
#include "aruwsrc/control/chassis/new_sentry/sentry_manual_drive_command.hpp"
#include "aruwsrc/robot/sentry/sentry_control_operator_interface.hpp"
#include "aruwsrc/control/chassis/new_sentry/sentry_chassis_rel_drive.hpp"
#include "aruwsrc/control/chassis/holonomic_chassis_subsystem.hpp"

using namespace aruwsrc::control::sentry;

namespace aruwsrc
{
namespace aruwsrc::control::sentry
{

SentryManualDriveCommand::SentryManualDriveCommand(
    tap::Drivers* drivers,
    SentryControlOperatorInterface* operatorInterface,
    chassis::HolonomicChassisSubsystem* chassis)
    : drivers(drivers),
      operatorInterface(operatorInterface),
      chassis(chassis)
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(chassis));
}

void SentryManualDriveCommand::initialize() {}

void SentryManualDriveCommand::execute()
{
    SentryChassisRelDrive::onExecute(operatorInterface, drivers, chassis);
}

void SentryManualDriveCommand::end(bool) { chassis->setZeroRPM(); }

bool SentryManualDriveCommand::isFinished() const { return false; }

}  // namespace chassis

}  // namespace aruwsrc
>>>>>>> origin/JoshuaT/#618/TurretManualControlCommand:aruw-mcb-project/src/aruwsrc/control/chassis/new_sentry/sentry_manual_drive_command.cpp
