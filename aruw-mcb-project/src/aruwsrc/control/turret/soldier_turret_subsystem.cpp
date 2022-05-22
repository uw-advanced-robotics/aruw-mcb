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

#include "soldier_turret_subsystem.hpp"

#include "aruwsrc/drivers.hpp"

namespace aruwsrc::control::turret
{
float SoldierTurretSubsystem::getWorldYaw() const { return getTurretMCB()->getYaw(); }

float SoldierTurretSubsystem::getWorldPitch() const { return getTurretMCB()->getPitch(); }

uint32_t SoldierTurretSubsystem::getLastMeasurementTimeMicros() const
{
    return getTurretMCB()->getIMUDataTimestamp();
}

}  // namespace aruwsrc::control::turret
