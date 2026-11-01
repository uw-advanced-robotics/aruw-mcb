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
#ifndef TRAJECTORY_IK_COMMAND_HPP_
#define TRAJECTORY_IK_COMMAND_HPP_

#include "tap/architecture/clock.hpp"

#include "aruwsrc/control/joint/joint_subsystem.hpp"
#include "aruwsrc/control/turret/turret_subsystem.hpp"
#include "aruwsrc/robot/engineer/wrist/wrist_subsystem.hpp"

#include "abstract_ik_command.hpp"
#include "trajectory_6d.hpp"

namespace aruwsrc::engineer::algorithms::inverse_kinematics
{
template <size_t LEN>
class TrajectoryIKCommand : public AbstractIKCommand
{
public:
    TrajectoryIKCommand(
        const tap::algorithms::transforms::Transform& chassisToBase,
        const tap::algorithms::transforms::Transform& followerToEndEffector,
        aruwsrc::control::turret::TurretSubsystem& turret,
        aruwsrc::control::joint::JointSubsystem& extension,
        aruwsrc::engineer::wrist::WristSubsystem& wrist,
        aruwsrc::control::turret::algorithms::TurretAxisControllerInterface<
            tap::algorithms::transforms::Axis::YAW>& yawController,
        aruwsrc::control::turret::algorithms::TurretAxisControllerInterface<
            tap::algorithms::transforms::Axis::PITCH>& pitchController,
        const Trajectory6D<LEN>& trajectory,
        bool holdFinalPose = false)
        : AbstractIKCommand(
              chassisToBase,
              followerToEndEffector,
              turret,
              extension,
              wrist,
              yawController,
              pitchController),
          trajectory(trajectory),
          holdFinalPose(holdFinalPose)
    {
    }

    const char* getName() const override { return "Trajectory IK Command"; }

    bool isReady() override
    {
        return yawController.isOnline() && pitchController.isOnline() && extension.isOnline() &&
               wrist.isOnline();
    }

    void initialize() override
    {
        yawController.initialize();
        pitchController.initialize();
        startTime = tap::arch::clock::getTimeMilliseconds();
        baseToFollowerDesired = trajectory.atTime(0.0f);
    }

    void end(bool interrupted) override
    {
        turret.yawMotor.setMotorOutput(0);
        turret.pitchMotor.setMotorOutput(0);
        if (interrupted || holdFinalPose)
        {
            // The extension and wrist run their controllers in subsystem refresh(). Stop chasing
            // the old trajectory setpoints when the operator leaves trajectory mode. Trigger
            // bindings call end(false) on switch release, even for an unfinished held trajectory.
            extension.setSetpoint(extension.getPosition());
            wrist.setSetpointTheta1(wrist.getTheta1());
            wrist.setSetpointTheta2(wrist.getTheta2());
            wrist.setSetpointTheta3(wrist.getTheta3());
        }
    }

    bool isFinished() const override
    {
        uint32_t now = tap::arch::clock::getTimeMilliseconds();
        // Holding keeps the controllers active and prevents whileTrue bindings from restarting
        // a completed trajectory. Leaving the mode interrupts the command instead.
        return !holdFinalPose && (now - startTime) / 1000.0f >= trajectory.waypoints.back().time;
    }

    void updateBaseToFollowerDesired() override
    {
        uint32_t now = tap::arch::clock::getTimeMilliseconds();

        baseToFollowerDesired = trajectory.atTime((now - startTime) / 1000.0f);
    }

private:
    const Trajectory6D<LEN>& trajectory;
    const bool holdFinalPose;

    uint32_t startTime = 0;
};  // class TrajectoryIKCommand

}  // namespace aruwsrc::engineer::algorithms::inverse_kinematics
#endif  // TRAJECTORY_IK_COMMAND_HPP_
