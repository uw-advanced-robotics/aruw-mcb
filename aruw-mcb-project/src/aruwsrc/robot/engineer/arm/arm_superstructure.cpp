/*
 * Copyright (c) 2024-2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "arm_superstructure.hpp"

namespace aruwsrc::engineer::arm
{
ArmSuperstructure::ArmSuperstructure(
    JointSubsystem* lift,
    JointSubsystem* reach,
    JointSubsystem* yaw,
    PitchSubsystem* pitch,
    JointSubsystem* roll)
    : liftJoint(lift),
      reachJoint(reach),
      yawJoint(yaw),
      pitchJoint(pitch),
      rollJoint(roll)
{
}

void ArmSuperstructure::goToPosition(float lift, float reach, float yaw, float pitch, float roll)
{
    liftJoint->setSetpoint(lift);
    reachJoint->setSetpoint(reach);
    yawJoint->setSetpoint(yaw);
    pitchJoint->setSetpoint(pitch);
    rollJoint->setSetpoint(roll);
}

bool ArmSuperstructure::atSetpoint()
{
    return liftJoint->atSetpoint() && reachJoint->atSetpoint() && yawJoint->atSetpoint() &&
           pitchJoint->atSetpoint() && rollJoint->atSetpoint();
}

bool ArmSuperstructure::isOnline()
{
    return liftJoint->isOnline() && reachJoint->isOnline() && yawJoint->isOnline() &&
           pitchJoint->isOnline() && rollJoint->isOnline();
}

}  // namespace aruwsrc::engineer::arm
