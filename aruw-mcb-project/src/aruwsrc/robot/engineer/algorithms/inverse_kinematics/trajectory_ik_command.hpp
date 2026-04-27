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
    template <size_t LEN>
    TrajectoryIKCommand(
        const tap::algorithms::transforms::Transform& worldToChassis,
        const tap::algorithms::transforms::Transform& cubeToEndEffector,
        aruwsrc::control::turret::TurretSubsystem& turret,
        aruwsrc::control::joint::JointSubsystem& extension,
        aruwsrc::engineer::wrist::WristSubsystem& wrist,
        aruwsrc::control::turret::algorithms::TurretAxisControllerInterface<
            aruwsrc::control::turret::algorithms::Axis::YAW>& yawController,
        aruwsrc::control::turret::algorithms::TurretAxisControllerInterface<
            aruwsrc::control::turret::algorithms::Axis::PITCH>& pitchController,
        const Trajectory6d<LEN>& trajectory)
        : AbstractIKCommand(
              chassisToWorld,
              cubeToEndEffector,
              turret,
              extension,
              wrist,
              yawController,
              pitchController)
    {
    }

    template <size_t LEN>
    const char* getName() const override
    {
        return "Manual IK Command";
    }

    template <size_t LEN>
    void initialize() override
    {
        startTime = tap::arch::clock::getTimeMilliseconds();
    }

    template <size_t LEN>
    void execute() override;

    template <size_t LEN>
    bool isFinished() const override
    {
        uint32_t now = tap::arch::clock::getTimeMilliseconds();
        return (now - startTime) / 1000.0f > trajectory.waypoints.back().time;
    }

    template <size_t LEN>
    tap::algorithms::transforms::Transform getBaseToFollowerDesired() override
    {
        uint32_t now = tap::arch::clock::getTimeMilliseconds();

        return trajectory.atTime((now - startTime) / 1000.0f);
    }

private:
    const Trajectory6d<LEN>& trajectory;

    uint32_t startTime;
};  // class TrajectoryIKCommand

}  // namespace aruwsrc::engineer::algorithms::inverse_kinematics
#endif  // TRAJECTORY_IK_COMMAND_HPP_
