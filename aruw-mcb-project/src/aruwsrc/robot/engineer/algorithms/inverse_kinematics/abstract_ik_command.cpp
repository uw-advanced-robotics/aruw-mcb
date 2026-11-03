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

#include <algorithm>
#include <cmath>

#include "tap/algorithms/math_user_utils.hpp"
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
          TURRET_PITCH_TO_EXTENSION_ZERO_POS.z() * TURRET_PITCH_TO_EXTENSION_ZERO_POS.z())
{
    addSubsystemRequirement(&turret);
    addSubsystemRequirement(&extension);
    addSubsystemRequirement(&wrist);
}

void AbstractIKCommand::execute()
{
    updateBaseToFollowerDesired();
    if (shouldRecalculateIk() && calculateDesiredSetpoints())
    {
        cachedIkBaseToFollowerDesired = baseToFollowerDesired;
        ikSolutionInitialized = true;
    }

    yawController.runController(0.002f, Angle(turretYawDesired));
    pitchController.runController(0.002f, Angle(turretPitchDesired));
    extension.setSetpoint(extensionDesired);
    wrist.setSetpointOrientation(extensionToWristDesired.getRotation());
    updateBaseToFollowerFromSetpoints();
}

bool AbstractIKCommand::shouldRecalculateIk() const
{
    const bool inputsChanged =
        !ikSolutionInitialized ||
        !staticTransformsClose(baseToFollowerDesired, cachedIkBaseToFollowerDesired) ||
        !staticTransformsClose(chassisToBase, cachedChassisToBase) ||
        !staticTransformsClose(followerToEndEffector, cachedFollowerToEndEffector);

    // recalculate if the inputs are changed and the inputs are finite
    return inputsChanged && staticTransformIsFinite(baseToFollowerDesired) &&
           staticTransformIsFinite(chassisToBase) && staticTransformIsFinite(followerToEndEffector);
}

bool AbstractIKCommand::calculateDesiredSetpoints()
{
    chassisToEndEffectorDesired =
        chassisToBase.composeStatic(baseToFollowerDesired).composeStatic(followerToEndEffector);

    // kept as a Transform because we want to compose it easily
    chassisToWristDesiredPos = chassisToEndEffectorDesired.composeStatic(END_EFFECTOR_TO_WRIST);

    Position turretYawToWristDesiredPos = EngineerTransforms::getHypotheticalChassisToTurretYaw(0)
                                              .getInverse()
                                              .composeStatic(chassisToWristDesiredPos)
                                              .getTranslation();

    // We know y component of turretPitchToExtension is 0 and thus doesn't affect yaw
    const float newTurretYawDesired =
        atan2f(turretYawToWristDesiredPos.y(), turretYawToWristDesiredPos.x());

    Position turretPitchToWristDesiredPos =
        EngineerTransforms::getHypotheticalChassisToTurretYaw(newTurretYawDesired)
            .composeStatic(EngineerTransforms::getHypotheticalTurretYawToTurretPitch(0))
            .getInverse()
            .composeStatic(chassisToWristDesiredPos)
            .getTranslation();

    float turretPitchToWristDesiredDistSq = turretPitchToWristDesiredPos.toVector().magnitudeSq();
    if (!std::isfinite(turretPitchToWristDesiredDistSq))
    {
        return false;
    }
    const float extensionRadicand =
        std::max(turretPitchToWristDesiredDistSq - turretPitchToExtensionZeroZ2, 0.0f);
    const float newExtensionDesired = std::clamp(
        sqrtf(extensionRadicand) - turretPitchToExtensionZeroX,
        extension.getLowerBound(),
        extension.getUpperBound());

    turretPitchToExtension =
        EngineerTransforms::getHypotheticalTurretPitchToExtension(newExtensionDesired);
    float x1 = turretPitchToWristDesiredPos.x();
    float y1 = turretPitchToWristDesiredPos.z();
    float x2 = turretPitchToExtension.getX();
    float y2 = turretPitchToExtension.getZ();
    const float newTurretPitchDesired = atan2f(x1 * y2 - y1 * x2, x1 * x2 + y1 * y2);

    Transform newExtensionToWristDesired =
        EngineerTransforms::getHypotheticalChassisToTurretYaw(newTurretYawDesired)
            .composeStatic(
                EngineerTransforms::getHypotheticalTurretYawToTurretPitch(newTurretPitchDesired))
            .composeStatic(
                EngineerTransforms::getHypotheticalTurretPitchToExtension(newExtensionDesired))
            .getInverse()
            .composeStatic(chassisToEndEffectorDesired)
            .composeStatic(END_EFFECTOR_TO_WRIST);

    if (!std::isfinite(newTurretYawDesired) || !std::isfinite(newTurretPitchDesired) ||
        !std::isfinite(newExtensionDesired) || !staticTransformIsFinite(newExtensionToWristDesired))
    {
        return false;
    }

    turretYawDesired = newTurretYawDesired;
    turretPitchDesired = newTurretPitchDesired;
    extensionDesired = newExtensionDesired;
    extensionToWristDesired = newExtensionToWristDesired;
    return true;
}

bool AbstractIKCommand::staticTransformsClose(const Transform& first, const Transform& second)
{
    const Vector translationDifference = first.getTranslation() - second.getTranslation();
    if (translationDifference.magnitudeSq() > TRANSFORM_EPSILON * TRANSFORM_EPSILON)
    {
        return false;
    }

    const auto& firstRotation = first.getRotation().matrix();
    const auto& secondRotation = second.getRotation().matrix();
    for (size_t i = 0; i < firstRotation.data.size(); ++i)
    {
        if (!tap::algorithms::compareFloatClose(
                firstRotation[i],
                secondRotation[i],
                TRANSFORM_EPSILON))
        {
            return false;
        }
    }
    return true;
}

bool AbstractIKCommand::staticTransformIsFinite(const Transform& transform)
{
    const Position& translation = transform.getTranslation();
    if (!std::isfinite(translation.x()) || !std::isfinite(translation.y()) ||
        !std::isfinite(translation.z()))
    {
        return false;
    }

    const auto& rotation = transform.getRotation().matrix();
    for (size_t i = 0; i < rotation.data.size(); ++i)
    {
        if (!std::isfinite(rotation[i]))
        {
            return false;
        }
    }
    return true;
}

void AbstractIKCommand::updateBaseToFollowerFromSetpoints()
{
    const float yawSetpoint = yawController.getSetpoint().getWrappedValue();
    const float pitchSetpoint = pitchController.getSetpoint().getWrappedValue();
    const float extensionSetpoint = extension.getSetpoint();
    const float wristTheta1Setpoint = wrist.getSetpointTheta1();
    const float wristTheta2Setpoint = wrist.getSetpointTheta2();
    const float wristTheta3Setpoint = wrist.getSetpointTheta3();

    const bool inputsFinite =
        staticTransformIsFinite(chassisToBase) && staticTransformIsFinite(followerToEndEffector) &&
        std::isfinite(yawSetpoint) && std::isfinite(pitchSetpoint) &&
        std::isfinite(extensionSetpoint) && std::isfinite(wristTheta1Setpoint) &&
        std::isfinite(wristTheta2Setpoint) && std::isfinite(wristTheta3Setpoint);
    if (!inputsFinite)
    {
        baseToFollowerDesired = cachedBaseToFollowerFromSetpoints;
        return;
    }

    const bool chassisChanged = !forwardKinematicsCacheInitialized ||
                                !staticTransformsClose(chassisToBase, cachedChassisToBase);
    const bool followerChanged =
        !forwardKinematicsCacheInitialized ||
        !staticTransformsClose(followerToEndEffector, cachedFollowerToEndEffector);
    const bool yawChanged =
        !forwardKinematicsCacheInitialized ||
        !tap::algorithms::compareFloatClose(yawSetpoint, cachedYawSetpoint, TRANSFORM_EPSILON);
    const bool pitchChanged =
        !forwardKinematicsCacheInitialized ||
        !tap::algorithms::compareFloatClose(pitchSetpoint, cachedPitchSetpoint, TRANSFORM_EPSILON);
    const bool extensionChanged =
        !forwardKinematicsCacheInitialized || !tap::algorithms::compareFloatClose(
                                                  extensionSetpoint,
                                                  cachedExtensionSetpoint,
                                                  TRANSFORM_EPSILON);
    const bool wristChanged = !forwardKinematicsCacheInitialized ||
                              !tap::algorithms::compareFloatClose(
                                  wristTheta1Setpoint,
                                  cachedWristTheta1Setpoint,
                                  TRANSFORM_EPSILON) ||
                              !tap::algorithms::compareFloatClose(
                                  wristTheta2Setpoint,
                                  cachedWristTheta2Setpoint,
                                  TRANSFORM_EPSILON) ||
                              !tap::algorithms::compareFloatClose(
                                  wristTheta3Setpoint,
                                  cachedWristTheta3Setpoint,
                                  TRANSFORM_EPSILON);

    const bool baseToYawChanged = chassisChanged || yawChanged;
    const bool yawToExtensionChanged = pitchChanged || extensionChanged;
    const bool extensionToFollowerChanged = wristChanged || followerChanged;

    if (baseToYawChanged)
    {
        cachedBaseToTurretYaw = chassisToBase.getInverse().composeStatic(
            EngineerTransforms::getHypotheticalChassisToTurretYaw(yawSetpoint));
    }
    if (yawToExtensionChanged)
    {
        cachedTurretYawToExtension =
            EngineerTransforms::getHypotheticalTurretYawToTurretPitch(pitchSetpoint)
                .composeStatic(
                    EngineerTransforms::getHypotheticalTurretPitchToExtension(extensionSetpoint));
    }
    if (extensionToFollowerChanged)
    {
        cachedExtensionToFollower = Transform(Position(), wrist.getSetpointOrientation())
                                        .composeStatic(WRIST_TO_END_EFFECTOR)
                                        .composeStatic(followerToEndEffector.getInverse());
    }
    if (baseToYawChanged || yawToExtensionChanged || extensionToFollowerChanged)
    {
        cachedBaseToFollowerFromSetpoints =
            cachedBaseToTurretYaw.composeStatic(cachedTurretYawToExtension)
                .composeStatic(cachedExtensionToFollower);
    }

    cachedChassisToBase = chassisToBase;
    cachedFollowerToEndEffector = followerToEndEffector;
    cachedYawSetpoint = yawSetpoint;
    cachedPitchSetpoint = pitchSetpoint;
    cachedExtensionSetpoint = extensionSetpoint;
    cachedWristTheta1Setpoint = wristTheta1Setpoint;
    cachedWristTheta2Setpoint = wristTheta2Setpoint;
    cachedWristTheta3Setpoint = wristTheta3Setpoint;
    forwardKinematicsCacheInitialized = true;

    baseToFollowerDesired = cachedBaseToFollowerFromSetpoints;
}

}  // namespace aruwsrc::engineer::algorithms::inverse_kinematics
