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
#ifndef MANUAL_IK_COMMAND_HPP_
#define MANUAL_IK_COMMAND_HPP_

#include "aruwsrc/robot/engineer/engineer_control_operator_interface.hpp"

#include "abstract_ik_command.hpp"

namespace aruwsrc::engineer::algorithms::inverse_kinematics
{
class ManualIKCommand : public AbstractIKCommand
{
public:
    ManualIKCommand(
        const aruwsrc::engineer::EngineerControlOperatorInterface& controlOperatorInterface,
        const tap::algorithms::transforms::Transform& chassisToBase,
        const tap::algorithms::transforms::Transform& cubeToEndEffector,
        const tap::algorithms::transforms::Transform& baseToEndEffector,
        aruwsrc::control::turret::TurretSubsystem& turret,
        aruwsrc::control::joint::JointSubsystem& extension,
        aruwsrc::engineer::wrist::WristSubsystem& wrist,
        aruwsrc::control::turret::algorithms::TurretAxisControllerInterface<
            tap::algorithms::transforms::Axis::YAW>& yawController,
        aruwsrc::control::turret::algorithms::TurretAxisControllerInterface<
            tap::algorithms::transforms::Axis::PITCH>& pitchController);

    const char* getName() const override { return "Manual IK Command"; }

    void initialize() override;

    tap::algorithms::transforms::Transform getBaseToFollowerDesired() override;

private:
    const aruwsrc::engineer::EngineerControlOperatorInterface& controlOperatorInterface;
    tap::algorithms::transforms::Transform baseToEndEffectorDesired;
    const tap::algorithms::transforms::Transform& baseToEndEffector;
};  // class ManualIKCommand

}  // namespace aruwsrc::engineer::algorithms::inverse_kinematics
#endif  // MANUAL_IK_COMMAND_HPP_
