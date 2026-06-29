/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef DRONE_BODY_CHASSIS_SUBSYSTEM_HPP_
#define DRONE_BODY_CHASSIS_SUBSYSTEM_HPP_

#include "tap/control/chassis/chassis_subsystem_interface.hpp"

namespace aruwsrc::drone
{
/**
 * Stub chassis for drone odometry.
 */
class DroneBodyChassisSubsystem : public tap::control::chassis::ChassisSubsystemInterface
{
public:
    explicit DroneBodyChassisSubsystem(tap::Drivers *drivers) : ChassisSubsystemInterface(drivers)
    {
    }

    int getNumChassisMotors() const override { return 0; }

    bool allMotorsOnline() const override { return true; }

    modm::Matrix<float, 3, 1> getActualVelocityChassisRelative() const override
    {
        const float values[] = {0.0f, 0.0f, 0.0f};
        return modm::Matrix<float, 3, 1>(values);
    }

    void initialize() override {}

    void refresh() override {}

    const char *getName() const override { return "DroneBody"; }
};
}  // namespace aruwsrc::drone

#endif  // DRONE_BODY_CHASSIS_SUBSYSTEM_HPP_
