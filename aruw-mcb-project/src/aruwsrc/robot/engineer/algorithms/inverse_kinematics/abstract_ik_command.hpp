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

#ifndef ABSTRACT_IK_COMMAND_HPP_
#define ABSTRACT_IK_COMMAND_HPP_

#include "tap/algorithms/transforms/transform.hpp"
#include "tap/control/command.hpp"

#include "aruwsrc/control/turret/algorithms/turret_controller_interface.hpp"

namespace aruwsrc::control::turret
{
class TurretSubsystem;
}
namespace aruwsrc::control::joint
{
class JointSubsystem;
}
namespace aruwsrc::engineer::wrist
{
class WristSubsystem;
}

namespace aruwsrc::engineer::algorithms::inverse_kinematics
{
/**
 * Abstract class that handles inverse kinematic operations. It expects subclasses to define "base"
 * and "follower" frames. The `Transform` between which will used to solve for joint positions. For
 * example, to place the cube at a specific position/orientation in world frame, we define the base
 * frame to be the world frame, and the follower frame to be the cube frame, finally supplying the
 * desired worldToCube `Transform`.
 *
 * @param chassisToBase `Transform` between the chassis and the defined "base" frame
 * @param followerToEndEffector `Transform` between the  defined "follower" frame and the end
 * effector
 *
 * @note The "follower" frame should be at or beyond the end effector in the kinematic tree.
 *
 * @warning The "base" frame can be placed beyond the chassis, however be wary since this class
 * always reformulates the problem as a chassisToEndEffector ik problem, it does not isolate
 * requested motion to within the bounds of the base and follower frames and can generate unstable
 * behavior. For example, if the turretYawToEndEffector transform is supplied with a y component
 * that cannot be achieved with the wrist alone, it will rotate the turret yaw to help. Since the
 * desired transform does not change wrt turret yaw, this won't actually help and it will not notice
 * that it already did this the next loop so it will continue to command the turret to rotate,
 * causing it to spin out of control.
 */
class AbstractIKCommand : public tap::control::Command
{
public:
    AbstractIKCommand(
        const tap::algorithms::transforms::Transform& chassisToBase,
        const tap::algorithms::transforms::Transform& followerToEndEffector,
        aruwsrc::control::turret::TurretSubsystem& turret,
        aruwsrc::control::joint::JointSubsystem& extension,
        aruwsrc::engineer::wrist::WristSubsystem& wrist,
        aruwsrc::control::turret::algorithms::TurretAxisControllerInterface<
            tap::algorithms::transforms::Axis::YAW>& yawController,
        aruwsrc::control::turret::algorithms::TurretAxisControllerInterface<
            tap::algorithms::transforms::Axis::PITCH>& pitchController);

    virtual void initialize() override {}

    virtual void execute() override;

    void end(bool) override {}

    virtual bool isFinished() const override { return false; }

    virtual void updateBaseToFollowerDesired() = 0;

protected:
    tap::algorithms::transforms::Transform baseToFollowerDesired;
    const tap::algorithms::transforms::Transform &chassisToBase, followerToEndEffector;
    aruwsrc::control::turret::TurretSubsystem& turret;
    aruwsrc::control::joint::JointSubsystem& extension;
    aruwsrc::engineer::wrist::WristSubsystem& wrist;
    aruwsrc::control::turret::algorithms::TurretAxisControllerInterface<
        tap::algorithms::transforms::Axis::YAW>& yawController;
    aruwsrc::control::turret::algorithms::TurretAxisControllerInterface<
        tap::algorithms::transforms::Axis::PITCH>& pitchController;

private:
    const float turretPitchToExtensionZeroX, turretPitchToExtensionZeroZ2;
    tap::algorithms::transforms::Transform chassisToEndEffectorDesired, chassisToWristDesiredPos,
        turretPitchToExtension, extensionToWristDesired;
    float turretYawDesired, turretPitchDesired, extensionDesired;
};

}  // namespace aruwsrc::engineer::algorithms::inverse_kinematics
#endif  // ABSTRACT_IK_COMMAND_HPP_