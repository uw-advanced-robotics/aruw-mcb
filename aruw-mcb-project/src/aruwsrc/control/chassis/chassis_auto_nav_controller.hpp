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
#include "aruwsrc/communication/serial/vision_coprocessor.hpp"
#include "aruwsrc/control/chassis/holonomic_chassis_subsystem.hpp"
#include "aruwsrc/control/chassis/sentry/sentry_beyblade_config.hpp"
#include "aruwsrc/robot/sentry/sentry_beyblade_command.hpp"

namespace aruwsrc::chassis
{

class ChassisAutoNavController
{
public:
    // TODO: clean up what should be fields and what should be passed into runController
    inline ChassisAutoNavController(
        aruwsrc::chassis::HolonomicChassisSubsystem& chassis,
        aruwsrc::algorithms::AutoNavPath& path,
        aruwsrc::serial::VisionCoprocessor& visionCoprocessor,
        tap::Drivers& drivers,
        const Transform& worldToChassis,
        const aruwsrc::sentry::SentryBeybladeCommand::SentryBeybladeConfig config)
        : chassis(chassis),
          path(path),
          lastSetPoint(Position(-1, -1, 0)),
          visionCoprocessor(visionCoprocessor),
          drivers(drivers),
          worldToChassis(worldToChassis),
          config(config)
    {
    }

    void initialize();

    void runController(
        const float maxWheelSpeed,
        const bool movementEnabled,
        const bool beybladeEnabled);

    Position calculateSetPoint(
        Position current,
        float interpolationParameter,
        bool movementEnabled);

    static inline Position quadraticBezierInterpolation(Position a, Position b, Position c, float t)
    {
        return Position::interpolate(
            Position::interpolate(a, b, t),
            Position::interpolate(b, c, t),
            t);
    }
    // private:
    aruwsrc::chassis::HolonomicChassisSubsystem& chassis;
    aruwsrc::algorithms::AutoNavPath& path;
    Position lastSetPoint;
    aruwsrc::serial::VisionCoprocessor& visionCoprocessor;
    tap::Drivers& drivers;

    const Transform& worldToChassis;

    aruwsrc::sentry::SentryBeybladeCommand::SentryBeybladeConfig config;

    tap::arch::MilliTimeout pathTransitionTimeout;
    float rotationDirection;
    tap::algorithms::Ramp rotateSpeedRamp;

    float LOOKAHEAD_DISTANCE = 0.2f;
    uint32_t PATH_TRANSITION_TIME_MILLIS = 750;
};
}  // namespace aruwsrc::chassis

#endif  // CHASSIS_AUTO_NAV_CONTROLLER_HPP_