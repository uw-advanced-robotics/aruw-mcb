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
        tap::algorithms::transforms::Axis::YAW>& yawController,
    aruwsrc::control::turret::algorithms::TurretAxisControllerInterface<
        tap::algorithms::transforms::Axis::PITCH>& pitchController)
    : chassisToBase(chassisToBase),
      followerToEndEffector(followerToEndEffector),
      turret(turret),
      extension(extension),
      wrist(wrist),
      yawController(yawController),
      pitchController(pitchController),
      turretPitchToExtensionZeroX(TURRET_PITCH_TO_EXTENSION_ZERO_POS.x()),
      turretPitchToExtensionZeroZ2(
          TURRET_PITCH_TO_EXTENSION_ZERO_POS.z() * TURRET_PITCH_TO_EXTENSION_ZERO_POS.z()),
      chassisToBaseInverse(this->chassisToBase.getInverse()),
      followerToEndEffectorInverse(this->followerToEndEffector.getInverse()),
      chassisToTurretYawZeroInverse(
          EngineerTransforms::getHypotheticalChassisToTurretYaw(0).getInverse()),
      chassisToTurretYawScratch(EngineerTransforms::getHypotheticalChassisToTurretYaw(0)),
      turretYawToTurretPitchScratch(EngineerTransforms::getHypotheticalTurretYawToTurretPitch(0)),
      wristSetpointTransform(Position(), Orientation())
{
    addSubsystemRequirement(&turret);
    addSubsystemRequirement(&extension);
    addSubsystemRequirement(&wrist);
}

void AbstractIKCommand::execute()
{
    chassisToBaseInverse = chassisToBase.getInverse();
    updateBaseToFollowerDesired();
    chassisToEndEffectorDesired =
        chassisToBase.composeStatic(baseToFollowerDesired).composeStatic(followerToEndEffector);

    chassisToWristDesiredPos = chassisToEndEffectorDesired.composeStatic(END_EFFECTOR_TO_WRIST);
    Position turretYawToWristDesiredPos =
        chassisToTurretYawZeroInverse.composeStatic(chassisToWristDesiredPos).getTranslation();
    turretYawDesired = atan2f(turretYawToWristDesiredPos.y(), turretYawToWristDesiredPos.x());

    chassisToTurretYawScratch.updateRotation(0, 0, turretYawDesired);
    turretYawToTurretPitchScratch.updateRotation(0, 0, 0);
    Position turretPitchToWristDesiredPos = chassisToTurretYawScratch
                                                .composeStatic(turretYawToTurretPitchScratch)
                                                .getInverse()
                                                .composeStatic(chassisToWristDesiredPos)
                                                .getTranslation();

    float turretPitchToWristDesiredDistSq = turretPitchToWristDesiredPos.toVector().magnitudeSq();
    extensionDesired = sqrtf(fabs(turretPitchToWristDesiredDistSq - turretPitchToExtensionZeroZ2)) -
                       turretPitchToExtensionZeroX;

    turretPitchToExtension.updateTranslation(
        TURRET_PITCH_TO_EXTENSION_ZERO_POS + Vector(extensionDesired, 0, 0));
    float x1 = turretPitchToWristDesiredPos.x();
    float y1 = turretPitchToWristDesiredPos.z();
    float x2 = turretPitchToExtension.getX();
    float y2 = turretPitchToExtension.getZ();
    turretPitchDesired = atan2f(x1 * y2 - y1 * x2, x1 * x2 + y1 * y2);

    turretYawToTurretPitchScratch.updateRotation(0, turretPitchDesired, 0);
    extensionToWristDesired = chassisToTurretYawScratch
                                  .composeStatic(turretYawToTurretPitchScratch)
                                  .composeStatic(turretPitchToExtension)
                                  .getInverse()
                                  .composeStatic(chassisToEndEffectorDesired)
                                  .composeStatic(END_EFFECTOR_TO_WRIST);

    // Set the desired setpoints
    yawController.runController(0.002f, Angle(turretYawDesired));
    pitchController.runController(0.002f, Angle(turretPitchDesired));
    extension.setSetpoint(extensionDesired);
    wrist.setSetpointOrientation(extensionToWristDesired.getRotation());

    chassisToTurretYawScratch.updateRotation(0, 0, yawController.getSetpoint().getWrappedValue());
    turretYawToTurretPitchScratch.updateRotation(
        0,
        pitchController.getSetpoint().getWrappedValue(),
        0);
    turretPitchToExtension.updateTranslation(
        TURRET_PITCH_TO_EXTENSION_ZERO_POS + Vector(extension.getSetpoint(), 0, 0));
    wristSetpointTransform.updateRotation(wrist.getSetpointOrientation());
    baseToFollowerDesired = chassisToBaseInverse
                                .composeStatic(chassisToTurretYawScratch)
                                .composeStatic(turretYawToTurretPitchScratch)
                                .composeStatic(turretPitchToExtension)
                                .composeStatic(wristSetpointTransform)
                                .composeStatic(WRIST_TO_END_EFFECTOR)
                                .composeStatic(followerToEndEffectorInverse);
}

}  // namespace aruwsrc::engineer::algorithms::inverse_kinematics
