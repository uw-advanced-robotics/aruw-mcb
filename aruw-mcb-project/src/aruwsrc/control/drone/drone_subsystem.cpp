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

#include "aruwsrc\control\drone\mavsdk\plugins\telemetry\include\plugins\telemetry\telemetry.h"

namespace aruwsrc
{
namespace drone
{

DroneSubsystem::DroneSubsystem(Drivers* drivers) : tap::control::Subsystem(drivers)
{
    // this might be wrong
    droneController.add_serial_connection("serial://COM3:57600");

    std::shared_ptr<mavsdk::System> system = droneController.systems()[0];
    auto telemetry = mavsdk::Telemetry{system};
    telemetry.set_rate_position(0.5);
    auto thing = telemetry.subscribe_position_velocity_ned(
        [&](mavsdk::Telemetry::PositionVelocityNed positionVel)
        { currentPosition = positionVel.position; });
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
