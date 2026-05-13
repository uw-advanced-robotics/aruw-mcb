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

#include "tap/algorithms/wrapped_float.hpp"

#include "aruwsrc/control/joint/joint_subsystem.hpp"
#include "aruwsrc/control/turret/turret_subsystem.hpp"
#include "aruwsrc/robot/engineer/algorithms/engineer_kinematic_constants.hpp"
#include "aruwsrc/robot/engineer/algorithms/engineer_transforms.hpp"
#include "aruwsrc/robot/engineer/wrist/wrist_subsystem.hpp"

using namespace tap::algorithms;
using namespace tap::algorithms::transforms;

namespace aruwsrc::engineer::algorithms::inverse_kinematics
{
AbstractIKCommand::AbstractIKCommand(
    const Transform& chassisToBase,
    const Transform& followerToEndEffector,
    aruwsrc::control::turret::TurretSubsystem& turret,
    aruwsrc::control::joint::JointSubsystem& extension,
    aruwsrc::engineer::wrist::WristSubsystem& wrist,
    aruwsrc::control::turret::algorithms::TurretAxisControllerInterface<
        aruwsrc::control::turret::algorithms::Axis::YAW>& yawController,
    aruwsrc::control::turret::algorithms::TurretAxisControllerInterface<
        aruwsrc::control::turret::algorithms::Axis::PITCH>& pitchController)
    : chassisToBase(chassisToBase),
      followerToEndEffector(followerToEndEffector),
      turret(turret),
      extension(extension),
      wrist(wrist),
      yawController(yawController),
      pitchController(pitchController),
      turretPitchToExtensionZeroX(TURRET_PITCH_TO_EXTENSION_ZERO_POS.x()),
      turretPitchToExtensionZeroZ2(
          TURRET_PITCH_TO_EXTENSION_ZERO_POS.z() * TURRET_PITCH_TO_EXTENSION_ZERO_POS.z())
{
    addSubsystemRequirement(&turret);
    addSubsystemRequirement(&extension);
    addSubsystemRequirement(&wrist);
}

void AbstractIKCommand::execute()
{
    chassisToEndEffectorDesired = chassisToBase.composeStatic(getBaseToFollowerDesired())
                                      .composeStatic(followerToEndEffector);

    // kept as a Transform because we want to compose it easily
    chassisToWristDesiredPos = chassisToEndEffectorDesired.composeStatic(END_EFFECTOR_TO_WRIST);

    Position turretYawToWristDesiredPos = EngineerTransforms::getHypotheticalChassisToTurretYaw(0)
                                              .getInverse()
                                              .composeStatic(chassisToWristDesiredPos)
                                              .getTranslation();

    // We know y component of turretPitchToExtension is 0 and thus doesn't affect yaw
    turretYawDesired = atan2f(turretYawToWristDesiredPos.y(), turretYawToWristDesiredPos.x());

    Position turretPitchToWristDesiredPos =
        EngineerTransforms::getHypotheticalChassisToTurretYaw(turretYawDesired)
            .composeStatic(EngineerTransforms::getHypotheticalTurretYawToTurretPitch(0))
            .getInverse()
            .composeStatic(chassisToWristDesiredPos)
            .getTranslation();

    float turretPitchToWristDesiredDistSq = turretPitchToWristDesiredPos.toVector().magnitudeSq();
    extensionDesired = sqrtf(turretPitchToWristDesiredDistSq - turretPitchToExtensionZeroZ2) -
                       turretPitchToExtensionZeroX;

    turretPitchToExtension =
        EngineerTransforms::getHypotheticalTurretPitchToExtension(extensionDesired);
    float x1 = turretPitchToWristDesiredPos.x();
    float y1 = turretPitchToWristDesiredPos.z();
    float x2 = turretPitchToExtension.getX();
    float y2 = turretPitchToExtension.getZ();
    turretPitchDesired = atan2f(x1 * y2 - y1 * x2, x1 * x2 + y1 * y2);

    extensionToWristDesired =
        EngineerTransforms::getHypotheticalChassisToTurretYaw(turretYawDesired)
            .composeStatic(
                EngineerTransforms::getHypotheticalTurretYawToTurretPitch(turretPitchDesired))
            .composeStatic(
                EngineerTransforms::getHypotheticalTurretPitchToExtension(extensionDesired))
            .getInverse()
            .composeStatic(chassisToEndEffectorDesired)
            .composeStatic(END_EFFECTOR_TO_WRIST);

    // Set the desired setpoints
    yawController.runController(0.002f, Angle(turretYawDesired));
    pitchController.runController(0.002f, Angle(turretPitchDesired));
    extension.setSetpoint(extensionDesired);
    wrist.setSetpointOrientation(extensionToWristDesired.getRotation());
}

}  // namespace aruwsrc::engineer::algorithms::inverse_kinematics
