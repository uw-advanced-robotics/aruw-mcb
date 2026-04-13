/*
 * Copyright (c) 2025-2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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
#include "manual_ik_command.hpp"

#include "engineer_transforms.hpp"

using namespace tap::algorithms::transforms;

namespace aruwsrc::engineer::algorithms
{
ManualIKCommand::ManualIKCommand(
    const tap::algorithms::transforms::Transform& worldToChassis,
    aruwsrc::control::turret::TurretSubsystem& turret,
    aruwsrc::control::joint::JointSubsystem& extension,
    aruwsrc::engineer::wrist::WristSubsystem& wrist,
    aruwsrc::control::joint::JointSubsystem& roll,
    aruwsrc::control::turret::algorithms::TurretAxisControllerInterface<
        aruwsrc::control::turret::algorithms::Axis::YAW>& yawController,
    aruwsrc::control::turret::algorithms::TurretAxisControllerInterface<
        aruwsrc::control::turret::algorithms::Axis::PITCH>& pitchController)
    : AbstractIKCommand(
          worldToChassis,
          turret,
          extension,
          wrist,
          roll,
          yawController,
          pitchController),
      chassisToEEDesired(Transform::identity())
{
}

void ManualIKCommand::initialize()
{
    chassisToEEDesired =
        algorithms::EngineerTransforms::getHypotheticalChassisToTurretYaw(0)
            .composeStatic(algorithms::EngineerTransforms::getHypotheticalTurretYawToTurretPitch(0))
            .composeStatic(algorithms::EngineerTransforms::getHypotheticalTurretPitchToExtension(0))
            .composeStatic(algorithms::WRIST_ROLL_TO_END_EFFECTOR);
}

void ManualIKCommand::execute()
{
    //
}

}  // namespace aruwsrc::engineer::algorithms
