/*
 * Copyright (c) 2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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
#ifndef CHASSIS_AUTO_NAV_CONTROLLER_HPP_
#define CHASSIS_AUTO_NAV_CONTROLLER_HPP_

#include "tap/algorithms/ramp.hpp"
#include "tap/algorithms/transforms/position.hpp"
#include "tap/communication/serial/ref_serial_data.hpp"
#include "tap/drivers.hpp"

#include "aruwsrc/algorithms/auto_nav_path.hpp"
#include "aruwsrc/algorithms/interpolate.hpp"
#include "aruwsrc/control/chassis/beyblade_config.hpp"
#include "aruwsrc/control/chassis/holonomic_chassis_subsystem.hpp"

#include "aruwsrc/control/cap_bank/cap_bank_subsystem.hpp"

namespace aruwsrc::chassis
{
class ChassisAutoNavController
{
public:
    // how much farther ahead along the path the robot movement aims for
    const float LOOKAHEAD_DISTANCE = 0.2f;

    // how long the controller takes to smoothly transition to an updated path
    const uint32_t PATH_TRANSITION_TIME_MILLIS = 400;

    // distance from setpoint under which robot is considered "on target"
    const float POS_ERROR_THRESHOLD = 0.01;

    inline ChassisAutoNavController(
        tap::Drivers& drivers,
        aruwsrc::chassis::HolonomicChassisSubsystem& chassis,
        const Transform& worldToChassis,
        const aruwsrc::chassis::BeybladeConfig beybladeConfig,
        aruwsrc::control::capbank::CapBankSubsystem& capBankSubsystem)
        : chassis(chassis),
          lastSetPoint(Position(-1, -1, 0)),
          drivers(drivers),
          worldToChassis(worldToChassis),
          beybladeConfig(beybladeConfig),
           capBankSubsystem(capBankSubsystem)
    {}

    void initialize();

    void runController(
        const float maxWheelSpeed,
        const bool movementEnabled,
        const bool beybladeEnabled);

    Position calculateSetPoint(
        Position current,
        float interpolationParameter,
        bool movementEnabled);

    // Sets the maximum speed the chassis moves at, in units of Meters per Second
    inline void setDesiredSpeed(float speed) { this->desiredSpeed = speed; }

    inline void attachPath(aruwsrc::algorithms::AutoNavPath* path) { this->path = path; }

private:
    aruwsrc::chassis::HolonomicChassisSubsystem& chassis;
    aruwsrc::algorithms::AutoNavPath* path = nullptr;
    Position lastSetPoint;
    tap::Drivers& drivers;

    const Transform& worldToChassis;

    aruwsrc::chassis::BeybladeConfig beybladeConfig;

    tap::arch::MilliTimeout pathTransitionTimeout;
    float rotationDirection;
    tap::algorithms::Ramp rotateSpeedRamp;
    
    aruwsrc::control::capbank::CapBankSubsystem& capBankSubsystem;

    float desiredSpeed = 0;
};
}  // namespace aruwsrc::chassis

#endif  // CHASSIS_AUTO_NAV_CONTROLLER_HPP_
