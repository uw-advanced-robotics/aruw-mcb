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
#ifndef ARM_SUPERSTRUCTURE_HPP_
#define ARM_SUPERSTRUCTURE_HPP_

#include "joint_subsystem.hpp"
#include "pitch_subsystem.hpp"

namespace aruwsrc::engineer::arm
{

class ArmSuperstructure
{
public:
    // Takes in 5 joints, lift reach yaw pitch roll

    ArmSuperstructure(
        JointSubsystem* lift,
        JointSubsystem* reach,
        JointSubsystem* yaw,
        PitchSubsystem* pitch,
        JointSubsystem* roll);

    void goToPosition(float lift, float reach, float yaw, float pitch, float roll);

    bool reachedPosition();

    bool isOnline();

private:
    JointSubsystem* liftJoint;
    JointSubsystem* reachJoint;
    JointSubsystem* yawJoint;
    PitchSubsystem* pitchJoint;
    JointSubsystem* rollJoint;
};

}  // namespace aruwsrc::engineer::arm

#endif
