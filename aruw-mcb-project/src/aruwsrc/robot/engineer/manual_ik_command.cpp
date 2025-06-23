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

using namespace tap::algorithms::transforms;

namespace aruwsrc::engineer
{

ManualIKCommand::ManualIKCommand(
    aruwsrc::chassis::HolonomicChassisSubsystem& chassis,
    gantry::GantryLiftSubsystem& gantryLift,
    gantry::GantryExtensionSubsystem& gantryExtension,
    wrist::WristSubsystem& wrist,
    JointSubsystem& roll,
    const Transform& worldToChassis)
    : AbstractIKCommand(chassis, gantryLift, gantryExtension, wrist, roll, worldToChassis),
      chassisToEEDesired(Transform::identity())
{
}

void ManualIKCommand::initialize()
{
    chassisToEEDesired = Transform(
        WRIST_TO_EE_TRANSLATION + Vector(EXTENSION_BASE_OFFSET_M, 0, LIFT_BASE_OFFSET_M),
        Orientation(0, 0, 0));
}

void ManualIKCommand::execute()
{
    //
}

}  // namespace aruwsrc::engineer
