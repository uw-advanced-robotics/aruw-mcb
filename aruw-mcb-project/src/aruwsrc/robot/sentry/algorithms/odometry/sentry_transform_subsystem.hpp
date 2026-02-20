/*
 * Copyright (c) 2020-2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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
#ifndef SENTRY_TRANSFORM_SUBSYSTEM_HPP_
#define SENTRY_TRANSFORM_SUBSYSTEM_HPP_

#include "tap/control/subsystem.hpp"

#include "aruwsrc/communication/rtt/rtt_telemetry.hpp"

#include "sentry_transforms.hpp"

namespace aruwsrc::sentry::algorithms::odometry
{
class SentryTransformSubystem : public tap::control::Subsystem
{
public:
    SentryTransformSubystem(
        tap::Drivers& drivers,
        SentryTransforms& transformer,
        aruwsrc::communication::rtt::RttTelemetry* telemetry = nullptr)
        : tap::control::Subsystem(&drivers),
          transformer(transformer),
          telemetry(telemetry)
    {
    }

    inline void initialize() override { transformer.initialize(); };
    inline void refresh() override
    {
        transformer.updateTransforms();

        if (telemetry != nullptr)
        {
            const Transform& worldToChassis = transformer.getWorldToChassis();
            telemetry->logSignal("state:chassis:pos", worldToChassis.getX(), worldToChassis.getY());
            telemetry->logSignal(
                "state:chassis:vel",
                worldToChassis.getXVel(),
                worldToChassis.getYVel());
            telemetry->logSignal("state:chassis:yaw", worldToChassis.getYaw());
        }
    };

private:
    SentryTransforms& transformer;
    aruwsrc::communication::rtt::RttTelemetry* telemetry;
};

}  // namespace aruwsrc::sentry::algorithms::odometry

#endif  // SENTRY_TRANSFORM_SUBSYSTEM_HPP_
