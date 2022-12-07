/*
 * Copyright (c) 2020-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "drone_subsystem.hpp"

#include "aruwsrc/drivers.hpp"

#include "aruwsrc/control/drone/mavlink/common/mavlink_msg_position_target_local_ned.h"

namespace aruwsrc
{
namespace drone
{

DroneSubsystem::DroneSubsystem(Drivers* drivers) : ChassisSubsystemInterface(drivers), parser(drivers)
{

}

modm::Matrix<float, 3, 1> DroneSubsystem::getActualVelocityChassisRelative() const {
    modm::Matrix<float, 3, 1> velocity;
    velocity[0][0] = parser.position.vx;
    velocity[1][0] = parser.position.vy;
    velocity[2][0] = drivers->mpu6500.getGx();
    return velocity;
}

/**
 * For when transforms drop
 *
 * Translation3D DroneSubsystem::getDronePosition(){
 * return new Translation3d(currentPosition.north_m, currentPosition.east_m,
 * currentPosition.down_m);
 * }
 */

}  // namespace drone

}  // namespace aruwsrc
