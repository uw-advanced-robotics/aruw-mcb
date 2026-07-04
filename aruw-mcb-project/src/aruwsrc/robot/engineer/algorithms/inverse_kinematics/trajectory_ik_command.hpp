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
        const Trajectory6D<LEN>& trajectory)
        : AbstractIKCommand(
              chassisToBase,
              followerToEndEffector,
              turret,
              extension,
              wrist,
              yawController,
              pitchController),
          trajectory(trajectory)
    {
    }

    const char* getName() const override { return "Manual IK Command"; }

    void initialize() override { startTime = tap::arch::clock::getTimeMilliseconds(); }

    bool isFinished() const override
    {
        uint32_t now = tap::arch::clock::getTimeMilliseconds();
        return (now - startTime) / 1000.0f > trajectory.waypoints.back().time;
    }

    void updateBaseToFollowerDesired() override
    {
        uint32_t now = tap::arch::clock::getTimeMilliseconds();

        baseToFollowerDesired = trajectory.atTime((now - startTime) / 1000.0f);
    }

private:
    const Trajectory6D<LEN>& trajectory;

    uint32_t startTime;
};  // class TrajectoryIKCommand

}  // namespace aruwsrc::engineer::algorithms::inverse_kinematics
#endif  // TRAJECTORY_IK_COMMAND_HPP_
