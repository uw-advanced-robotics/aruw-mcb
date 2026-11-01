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
      followerToWristPos(
          followerToEndEffector.applyReverse(END_EFFECTOR_TO_WRIST.getTranslation())),
      followerToWristRotation(
          followerToEndEffector.getRotation().matrix() *
          END_EFFECTOR_TO_WRIST.getRotation().matrix()),
      wristToFollowerRotation(followerToWristRotation.transpose()),
      wristToFollowerPos(-(wristToFollowerRotation * followerToWristPos.coordinates()))
{
    addSubsystemRequirement(&turret);
    addSubsystemRequirement(&extension);
    addSubsystemRequirement(&wrist);
}

void AbstractIKCommand::execute()
{
    updateBaseToFollowerDesired();
    chassisToWristDesiredRotation =
        chassisToBase.getRotation().matrix() * baseToFollowerDesired.getRotation().matrix();
    chassisToWristDesiredPos =
        chassisToBase.applyReverse(baseToFollowerDesired.getTranslation()) +
        Vector(chassisToWristDesiredRotation * followerToWristPos.coordinates());
    chassisToWristDesiredRotation = chassisToWristDesiredRotation * followerToWristRotation;

    const float wristX = chassisToWristDesiredPos.x() - CHASSIS_TO_TURRET_YAW_POS.x();
    const float wristY = chassisToWristDesiredPos.y() - CHASSIS_TO_TURRET_YAW_POS.y();
    const float wristZ = chassisToWristDesiredPos.z() - CHASSIS_TO_TURRET_YAW_POS.z();

    // We know y component of turretPitchToExtension is 0 and thus doesn't affect yaw
    turretYawDesired = atan2f(wristY, wristX);
    const float sinYaw = sinf(turretYawDesired);
    const float cosYaw = cosf(turretYawDesired);

    // Rotate into the yaw frame, then subtract the pitch joint's fixed offset.
    const float x1 = cosYaw * wristX + sinYaw * wristY - TURRET_YAW_TO_TURRET_PITCH_POS.x();
    const float lateral = -sinYaw * wristX + cosYaw * wristY - TURRET_YAW_TO_TURRET_PITCH_POS.y();
    const float y1 = wristZ - TURRET_YAW_TO_TURRET_PITCH_POS.z();
    const float turretPitchToWristDesiredDistSq = x1 * x1 + lateral * lateral + y1 * y1;
    extensionDesired = sqrtf(turretPitchToWristDesiredDistSq - turretPitchToExtensionZeroZ2) -
                       turretPitchToExtensionZeroX;

    const float x2 = turretPitchToExtensionZeroX + extensionDesired;
    const float y2 = TURRET_PITCH_TO_EXTENSION_ZERO_POS.z();
    turretPitchDesired = atan2f(x1 * y2 - y1 * x2, x1 * x2 + y1 * y2);
    const float sinPitch = sinf(turretPitchDesired);
    const float cosPitch = cosf(turretPitchDesired);

    // Apply R_pitch^T * R_yaw^T directly to each column. Extension contributes no rotation.
    for (size_t col = 0; col < 3; ++col)
    {
        const float x = cosYaw * chassisToWristDesiredRotation.data[col] +
                        sinYaw * chassisToWristDesiredRotation.data[3 + col];
        const float y = -sinYaw * chassisToWristDesiredRotation.data[col] +
                        cosYaw * chassisToWristDesiredRotation.data[3 + col];
        const float z = chassisToWristDesiredRotation.data[6 + col];
        extensionToWristDesiredRotation.data[col] = cosPitch * x - sinPitch * z;
        extensionToWristDesiredRotation.data[3 + col] = y;
        extensionToWristDesiredRotation.data[6 + col] = sinPitch * x + cosPitch * z;
    }

    // Set the desired setpoints
    yawController.runController(0.002f, Angle(turretYawDesired));
    pitchController.runController(0.002f, Angle(turretPitchDesired));
    extension.setSetpoint(extensionDesired);
    wrist.setSetpointRotationMatrix(extensionToWristDesiredRotation);

    updateBaseToFollowerFromSetpoints();
}

void AbstractIKCommand::updateBaseToFollowerFromSetpoints()
{
    // Read back the actual setpoints: the controllers and wrist may have clamped the requests.
    const float yaw = yawController.getSetpoint().getWrappedValue();
    const float pitch = pitchController.getSetpoint().getWrappedValue();
    const float sinYaw = sinf(yaw), cosYaw = cosf(yaw);
    const float sinPitch = sinf(pitch), cosPitch = cosf(pitch);
    const float extensionX = TURRET_PITCH_TO_EXTENSION_ZERO_POS.x() + extension.getSetpoint();
    const float extensionY = TURRET_PITCH_TO_EXTENSION_ZERO_POS.y();
    const float extensionZ = TURRET_PITCH_TO_EXTENSION_ZERO_POS.z();

    const float pitchX =
        TURRET_YAW_TO_TURRET_PITCH_POS.x() + cosPitch * extensionX + sinPitch * extensionZ;
    const float pitchY = TURRET_YAW_TO_TURRET_PITCH_POS.y() + extensionY;
    const float pitchZ =
        TURRET_YAW_TO_TURRET_PITCH_POS.z() - sinPitch * extensionX + cosPitch * extensionZ;
    chassisToWristDesiredPos = Position(
        CHASSIS_TO_TURRET_YAW_POS.x() + cosYaw * pitchX - sinYaw * pitchY,
        CHASSIS_TO_TURRET_YAW_POS.y() + sinYaw * pitchX + cosYaw * pitchY,
        CHASSIS_TO_TURRET_YAW_POS.z() + pitchZ);

    chassisToWristDesiredRotation = CMSISMat<3, 3>(
        {cosYaw * cosPitch,
         -sinYaw,
         cosYaw * sinPitch,
         sinYaw * cosPitch,
         cosYaw,
         sinYaw * sinPitch,
         -sinPitch,
         0,
         cosPitch});
    extensionToWristDesiredRotation = wrist.getSetpointRotationMatrix();
    chassisToWristDesiredRotation = chassisToWristDesiredRotation * extensionToWristDesiredRotation;
    chassisToWristDesiredPos =
        chassisToWristDesiredPos +
        Vector(chassisToWristDesiredRotation * wristToFollowerPos.coordinates());
    chassisToWristDesiredRotation = chassisToWristDesiredRotation * wristToFollowerRotation;

    baseToFollowerDesired.updateTranslation(chassisToBase.applyForward(chassisToWristDesiredPos));
    extensionToWristDesiredRotation =
        chassisToBase.getRotation().matrixT() * chassisToWristDesiredRotation;
    baseToFollowerDesired.updateRotation(Orientation(extensionToWristDesiredRotation));
    // Like composeStatic(), clear all input derivatives, including those below isDynamic()'s
    // tolerance. The usual static-input path does not need to rewrite them.
    if (baseToFollowerDesired.isDynamic() || baseToFollowerDesired.getXVel() != 0 ||
        baseToFollowerDesired.getYVel() != 0 || baseToFollowerDesired.getZVel() != 0 ||
        baseToFollowerDesired.getXAcc() != 0 || baseToFollowerDesired.getYAcc() != 0 ||
        baseToFollowerDesired.getZAcc() != 0 || baseToFollowerDesired.getRollVelocity() != 0 ||
        baseToFollowerDesired.getPitchVelocity() != 0 ||
        baseToFollowerDesired.getYawVelocity() != 0)
    {
        baseToFollowerDesired.updateVelocity(0, 0, 0);
        baseToFollowerDesired.updateAcceleration(0, 0, 0);
        baseToFollowerDesired.updateAngularVelocity(0, 0, 0);
    }
}

}  // namespace aruwsrc::engineer::algorithms::inverse_kinematics
