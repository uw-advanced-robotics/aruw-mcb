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

#ifndef STANDARD_AND_HERO_TRANSFORMER_SUBSYSTEM_HPP_
#define STANDARD_AND_HERO_TRANSFORMER_SUBSYSTEM_HPP_

#include "tap/control/subsystem.hpp"

#include "aruwsrc/communication/serial/rtt_telemetry.hpp"

#include "standard_and_hero_transformer.hpp"

namespace aruwsrc::algorithms::odometry::transforms
{
/**
 * @brief a convenience subsystem for updating a StandardAndHeroTransformer each control loop
 */
class StandardAnderHeroTransformerSubsystem : public tap::control::Subsystem
{
public:
    StandardAnderHeroTransformerSubsystem(
        tap::Drivers& drivers,
        StandardAndHeroTransformer& transformer,
        aruwsrc::communication::serial::RttTelemetry* telemetry = nullptr)
        : tap::control::Subsystem(&drivers),
          transformer(transformer),
          telemetry(telemetry)
    {
    }

    inline void initialize() override{};
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
    const char* getName() const { return "Standard and hero transformer subsystem"; }

private:
    StandardAndHeroTransformer& transformer;
    aruwsrc::communication::serial::RttTelemetry* telemetry;
};

}  // namespace aruwsrc::algorithms::odometry::transforms

#endif  // STANDARD_AND_HERO_TRANSFORMER_SUBSYSTEM_HPP_
