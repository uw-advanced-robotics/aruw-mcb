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

#ifndef STANDARD_TURRET_SUBSYSTEM_HPP_
#define STANDARD_TURRET_SUBSYSTEM_HPP_

#include "aruwsrc/control/turret/turret_subsystem.hpp"

namespace aruwsrc::control::turret
{
/**
 * Turret subsystem for the Standard.
 */
class StandardTurretSubsystem final : public TurretSubsystem
{
public:
    using TurretSubsystem::TurretSubsystem;

    inline const tap::algorithms::transforms::Position getTurretOffset() const override
    {
        return tap::algorithms::transforms::Position(0, 0, 0);
    }

    inline float getPitchOffset() const override { return 0; }
};  // class StandardTurretSubsystem

}  // namespace aruwsrc::control::turret

#endif  // STANDARD_TURRET_SUBSYSTEM_HPP_
