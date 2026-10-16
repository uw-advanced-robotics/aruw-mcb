/*
 * Copyright (c) 2021-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef ENGINEER_TURRET_SUBSYSTEM_HPP_
#define ENGINEER_TURRET_SUBSYSTEM_HPP_

#include "aruwsrc/control/turret/robot_turret_subsystem.hpp"

namespace aruwsrc::engineer
{
/**
 * Turret subsystem for the Engineer.
 */
class EngineerTurretSubsystem final : public control::turret::RobotTurretSubsystem
{
    using control::turret::RobotTurretSubsystem::RobotTurretSubsystem;
    float getWorldYaw() const override;
    float getWorldPitch() const override;
    modm::Vector3f getTurretOffset() const override { return modm::Vector3f(0, 0, 0); };
    float getPitchOffset() const override { return 0; };
};  // class EngineerTurretSubsystem

}  // namespace aruwsrc::engineer
#endif  // ENGINEER_TURRET_SUBSYSTEM_HPP_
