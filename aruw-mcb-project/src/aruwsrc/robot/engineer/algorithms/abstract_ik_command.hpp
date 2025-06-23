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

#include "../gantry/gantry_extension_subsystem.hpp"
#include "../gantry/gantry_lift_subsystem.hpp"
#include "../joint_subsystem.hpp"
#include "../wrist/wrist_subsystem.hpp"
#include "aruwsrc/control/chassis/holonomic_chassis_subsystem.hpp"

namespace aruwsrc::engineer::algorithms
{

class AbstractIKCommand : public tap::control::Command
{
public:
    AbstractIKCommand(
        aruwsrc::chassis::HolonomicChassisSubsystem& chassis,
        aruwsrc::engineer::gantry::GantryLiftSubsystem& gantryLift,
        aruwsrc::engineer::gantry::GantryExtensionSubsystem& gantryExtension,
        aruwsrc::engineer::wrist::WristSubsystem& wrist,
        aruwsrc::engineer::JointSubsystem& roll,
        const tap::algorithms::transforms::Transform& worldToChassis);
    virtual ~AbstractIKCommand() override;

    virtual void initialize() override = 0;

    virtual void execute() override = 0;

    void end(bool) override {};

    bool isFinished() const override { return false; }

    virtual tap::algorithms::transforms::Transform getWorldToEEDesired() = 0;

protected:
    aruwsrc::chassis::HolonomicChassisSubsystem& chassis;
    aruwsrc::engineer::gantry::GantryLiftSubsystem& gantryLift;
    aruwsrc::engineer::gantry::GantryExtensionSubsystem& gantryExtension;
    aruwsrc::engineer::wrist::WristSubsystem& wrist;
    aruwsrc::engineer::JointSubsystem& roll;
    const tap::algorithms::transforms::Transform& worldToChassis;

    const tap::algorithms::transforms::Position WRIST_TO_EE_TRANSLATION =
        tap::algorithms::transforms::Position(0.23757f, 0, 0.02585);
    static constexpr float EXTENSION_BASE_OFFSET_M = 0.26353f;
    static constexpr float LIFT_BASE_OFFSET_M = 0.45669f;  // +-10mm
};

}  // namespace aruwsrc::engineer::algorithms
#endif  // ABSTRACT_IK_COMMAND_HPP_   `