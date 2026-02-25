/*
 * Copyright (c) 2026 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef CONSTANT_FIRE_RATE_AGITATOR_COMMAND_HPP_
#define CONSTANT_FIRE_RATE_AGITATOR_COMMAND_HPP_

#include "tap/control/setpoint/commands/move_integral_command.hpp"

namespace aruwsrc::control::agitator
{
/**
 * A move-integral-compatible command that continuously spins the agitator at a velocity derived
 * from target shot rate and agitator pocket count.
 *
 * This command intentionally does not finish when an integral target is reached.
 */
class ConstantFireRateAgitatorCommand : public tap::control::setpoint::MoveIntegralCommand
{
public:
    struct Config
    {
        float targetShotRateRps;
        int agitatorPocketCount;
    };

    ConstantFireRateAgitatorCommand(
        tap::control::setpoint::IntegrableSetpointSubsystem& integrableSetpointSubsystem,
        const Config& config)
        : tap::control::setpoint::MoveIntegralCommand(
              integrableSetpointSubsystem,
              makeMoveIntegralConfig(config)),
          desiredSetpoint(
              config.targetShotRateRps * (M_TWOPI / static_cast<float>(config.agitatorPocketCount)))
    {
    }

    const char* getName() const override { return "constant fire rate agitator command"; }

    void initialize() override { integrableSetpointSubsystem.setSetpoint(desiredSetpoint); }

    void execute() override {}

    void end(bool) override { integrableSetpointSubsystem.setSetpoint(0); }

    bool isFinished() const override
    {
        return integrableSetpointSubsystem.isJammed() || !integrableSetpointSubsystem.isOnline();
    }

private:

    float actualFireRate;

    static tap::control::setpoint::MoveIntegralCommand::Config makeMoveIntegralConfig(
        const Config& config)
    {
        const float setpoint =
            config.targetShotRateRps * (M_TWOPI / static_cast<float>(config.agitatorPocketCount));
        return tap::control::setpoint::MoveIntegralCommand::Config{
            setpoint,
            setpoint,
            0.0f,
        };
    }

    float desiredSetpoint;
};
}  // namespace aruwsrc::control::agitator

#endif  // CONSTANT_FIRE_RATE_AGITATOR_COMMAND_HPP_
