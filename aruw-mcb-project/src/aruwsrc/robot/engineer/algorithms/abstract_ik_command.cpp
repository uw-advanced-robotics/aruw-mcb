/*
 * Copyright (c) 2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "abstract_ik_command.hpp"

using namespace aruwsrc::engineer;
using namespace aruwsrc::chassis;

using namespace tap::algorithms::transforms;

namespace aruwsrc::engineer::algorithms
{

AbstractIKCommand::AbstractIKCommand(
    aruwsrc::chassis::HolonomicChassisSubsystem& chassis,
    gantry::GantryLiftSubsystem& gantryLift,
    gantry::GantryExtensionSubsystem& gantryExtension,
    wrist::WristSubsystem& wrist,
    JointSubsystem& roll,
    const Transform& worldToChassis)
    : chassis(chassis),
      gantryLift(gantryLift),
      gantryExtension(gantryExtension),
      wrist(wrist),
      roll(roll),
      worldToChassis(worldToChassis)
{
    // addSubsystemRequirement(&chassis);
    addSubsystemRequirement(&gantryLift);
    addSubsystemRequirement(&gantryExtension);
    addSubsystemRequirement(&wrist);
    addSubsystemRequirement(&roll);
}
AbstractIKCommand::~AbstractIKCommand() = default;

void AbstractIKCommand::initialize() {}

void AbstractIKCommand::execute()
{
    Transform worldToEEDesired = getWorldToEEDesired();

    Transform chassisToEEDesired = worldToChassis.getInverse().composeStatic(worldToEEDesired);

    Orientation wristOrientationDesired = chassisToEEDesired.getRotation();

    auto mat = wristOrientationDesired.matrix().data;

    float wristRollDesired = atan2f(-mat[1 * 3 + 2], mat[1 * 3 + 1]);
    float wristPitchDesired = asinf(mat[2 * 3 + 0]);
    float wristYawDesired = atan2f(-mat[2 * 3 + 0], mat[0 * 3 + 0]);

    Transform wristToEEDesired =
        wrist.computeWristOrientation(wristYawDesired, wristPitchDesired)
            .composeStatic(Transform(WRIST_TO_EE_TRANSLATION, Orientation(wristRollDesired, 0, 0)));

    Transform chassisToWristDesired =
        chassisToEEDesired.composeStatic(wristToEEDesired.getInverse());

    // Set the desired setpoints
    roll.setSetpoint(wristRollDesired);
    wrist.setSetpointPitch(wristPitchDesired);
    wrist.setSetpointYaw(wristYawDesired);
    gantryExtension.setSetpoint(chassisToWristDesired.getX() - EXTENSION_BASE_OFFSET_M);
    gantryLift.setSetpoint(chassisToWristDesired.getZ() - LIFT_BASE_OFFSET_M);
    // todo: doesn't handle chassis y axis control
}

}  // namespace aruwsrc::engineer::algorithms
