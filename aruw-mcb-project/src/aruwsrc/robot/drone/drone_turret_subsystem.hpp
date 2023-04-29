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

#ifndef DRONE_TURRET_SUBSYSTEM_HPP_
#define DRONE_TURRET_SUBSYSTEM_HPP_

#include "aruwsrc/control/turret/turret_orientation_interface.hpp"
#include "aruwsrc/control/turret/turret_subsystem.hpp"

namespace aruwsrc::control::turret::algorithms
{
class TurretPitchControllerInterface;
class TurretYawControllerInterface;
}  // namespace aruwsrc::control::turret::algorithms

namespace aruwsrc::control::turret
{
/**
 * Turret subsystem for the Standard.
 */
class DroneTurretSubsystem final : public aruwsrc::control::turret::TurretSubsystem,
                                   public aruwsrc::control::turret::TurretOrientationInterface
{
    using TurretSubsystem::TurretSubsystem;
    float getWorldYaw() const override;
    float getWorldPitch() const override;
    modm::Vector3f getTurretOffset() const override { return modm::Vector3f(0, 0, 0); };
    float getPitchOffset() const override { return 0; };
};  // class StandardTurretSubsystem

}  // namespace aruwsrc::control::turret

#endif  // DRONE_TURRET_SUBSYSTEM_HPP_
