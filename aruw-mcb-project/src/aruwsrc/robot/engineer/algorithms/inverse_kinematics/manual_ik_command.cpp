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

#include "aruwsrc/robot/engineer/algorithms/engineer_transforms.hpp"

using namespace tap::algorithms::transforms;

namespace aruwsrc::engineer::algorithms::inverse_kinematics
{
ManualIKCommand::ManualIKCommand(
    const aruwsrc::engineer::EngineerControlOperatorInterface& controlOperatorInterface,
    const Transform& chassisToBase,
    const Transform& cubeToEndEffector,
    const Transform& baseToEndEffector,
    aruwsrc::control::turret::TurretSubsystem& turret,
    aruwsrc::control::joint::JointSubsystem& extension,
    aruwsrc::engineer::wrist::WristSubsystem& wrist,
    aruwsrc::control::turret::algorithms::TurretAxisControllerInterface<
        aruwsrc::control::turret::algorithms::Axis::YAW>& yawController,
    aruwsrc::control::turret::algorithms::TurretAxisControllerInterface<
        aruwsrc::control::turret::algorithms::Axis::PITCH>& pitchController)
    : AbstractIKCommand(
          chassisToBase,
          cubeToEndEffector,
          turret,
          extension,
          wrist,
          yawController,
          pitchController),
      controlOperatorInterface(controlOperatorInterface),
      baseToEndEffectorDesired(),
      baseToEndEffector(baseToEndEffector)
{
}

void ManualIKCommand::initialize() { baseToEndEffectorDesired = baseToEndEffector; }

Transform ManualIKCommand::getBaseToFollowerDesired()
{
    Vector endEffectorPrevToEndEffectorNextTrans = Vector(
                                                       controlOperatorInterface.getIKVelX(),
                                                       controlOperatorInterface.getIKVelY(),
                                                       controlOperatorInterface.getIKVelZ()) *
                                                   0.002f;
    Orientation endEffectorPrevToEndEffectorNextRot(
        controlOperatorInterface.getIKVelRoll() * 0.002f,
        controlOperatorInterface.getIKVelPitch() * 0.002f,
        controlOperatorInterface.getIKVelYaw() * 0.002f);

    // we apply translation in world frame for user control intuitiveness
    baseToEndEffectorDesired.updateTranslation(
        baseToEndEffectorDesired.getTranslation() + endEffectorPrevToEndEffectorNextTrans);
    baseToEndEffectorDesired.updateRotation(
        baseToEndEffectorDesired.getRotation().compose(endEffectorPrevToEndEffectorNextRot));

    return baseToEndEffectorDesired;
}

}  // namespace aruwsrc::engineer::algorithms::inverse_kinematics
