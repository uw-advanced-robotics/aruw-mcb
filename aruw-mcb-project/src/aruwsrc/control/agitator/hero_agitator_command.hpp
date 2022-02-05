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

#ifndef HERO_AGITATOR_COMMAND_HPP_
#define HERO_AGITATOR_COMMAND_HPP_

#include "tap/control/comprised_command.hpp"
#include "tap/control/setpoint/commands/move_command.hpp"
#include "tap/control/setpoint/commands/move_unjam_comprised_command.hpp"

#include "aruwsrc/control/launcher/friction_wheel_subsystem.hpp"

using tap::gpio::Digital;

namespace aruwsrc
{
class Drivers;
}

namespace aruwsrc::agitator
{
class AgitatorSubsystem;

/**
 * A command that launches a projectile and loads a new ball into the kicker.
 */
class HeroAgitatorCommand : public tap::control::ComprisedCommand
{
public:
    /**
     * Agitator rotation configuration parameters for this command.
     */
    struct Config
    {
        /** Amount to rotate the kicker when shooting a projectile (radians). */
        float kickerShootRotateAngle;
        /** Time it takes to rotate the kicker when shooting a projectile (ms). */
        uint32_t kickerShootRotateTime;
        /** Amount to rotate the kicker when loading a projectile (radians). */
        float kickerLoadRotateAngle;
        /** Amount to rotate the waterwheel when loading a projectile (radians). */
        float waterwheelLoadRotateAngle;
        /** Time it takes to rotate the waterwheel and kicker when loading a projectile (ms). */
        uint32_t loadRotateTime;
        /** Max unjam angle of the waterwheel if jamming happens */
        float waterwheelMaxUnjamAngle;
        /** Whether or not the command should heat limit. */
        bool heatLimiting;
        /** Minimum difference between the current heat and max 42mm heat required for this command
         * to shoot a projectile when heat limiting enabled. */
        uint16_t heatLimitBuffer;
    };

    /**
     * Constructs a HeroAgitatorCommand with the given agitator motors, flywheels, and
     * agitator rotation configuration params.
     */
    HeroAgitatorCommand(
        aruwsrc::Drivers* drivers,
        AgitatorSubsystem* kickerAgitator,
        AgitatorSubsystem* waterwheelAgitator,
        const aruwsrc::control::launcher::FrictionWheelSubsystem* frictionWheels,
        const Config& config);

    /**
     * @return Whether the agitator subsystems are online and heat is below
     * the heat limit buffer, indicating that the command is ready.
     */
    bool isReady() override;

    /**
     * @return Whether the command has finished running.
     */
    bool isFinished() const override;

    /**
     * Initializes the command. Spins the kicker to shoot the ball if the
     * limit switch is triggered. Spins the agitator and kicker to load a ball
     * otherwise.
     */
    void initialize() override;

    /**
     * Schedules commands to load a ball if the kicker has finished shooting a
     * ball or if the load commands have finished but the limit switch isn't pressed.
     * Enters the finished state if the limit switch is pressed.
     */
    void execute() override;

    /**
     * Removes all commands from scheduler, resets command to initial state.
     *
     * @param interrupted Whether the command was ended due to an interruption.
     */
    void end(bool interrupted) override;

    /**
     * @return Name of command
     */
    const char* getName() const override { return "hero agitator shoot"; }

private:
    tap::control::setpoint::MoveCommand kickerFireCommand;
    tap::control::setpoint::MoveCommand kickerLoadCommand;
    tap::control::setpoint::MoveUnjamComprisedCommand waterwheelLoadCommand;

    enum HeroAgitatorState
    {
        SHOOTING,
        LOAD,
        FINISHED
    };

    aruwsrc::Drivers* drivers;
    AgitatorSubsystem* kickerAgitator;
    AgitatorSubsystem* waterwheelAgitator;
    const aruwsrc::control::launcher::FrictionWheelSubsystem* frictionWheels;
    HeroAgitatorState currState;
    bool heatLimiting;
    uint16_t heatLimitBuffer;
    uint16_t startingHeat;

    /**
     * Enters the loading state and schedules commands to rotate the kicker
     * and agitator.
     */
    void beginLoading();
};  // class HeroAgitatorCommand

}  // namespace aruwsrc::agitator

#endif  // HERO_AGITATOR_COMMAND_HPP_
