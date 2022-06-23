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

#ifndef LAUNCH_DART_COMPRISED_COMMAND_HPP_
#define LAUNCH_DART_COMPRISED_COMMAND_HPP_

#include "aruwsrc/drivers.hpp"
#include "tap/control/comprised_command.hpp"
#include "tap/control/setpoint/commands/move_absolute_command.hpp"

#include "aruwsrc/control/turret/stepper_turret_subsystem.hpp"
#include "aruwsrc/control/turret/user/stepper_motor_turret_control_command.hpp"
#include "aruwsrc/control/agitator/agitator_subsystem.hpp"

namespace aruwsrc::control::dart 
{
struct LaunchDartComprisedCommandConfig
{
    float agitatorSetpoint;  // Radians
    float agitatorAngularSpeed;  // Radians/Second
    float agitatorSetpointTolerance;  // Radians
};
struct LaunchDartOffsetConfig
{
    int32_t yawOffsetSteps;
    int32_t pitchOffsetSteps;
    float timeDelayAfterMoving; // Milliseconds
};

/**
 * Comprised Command which launches two darts on the dart launcher
 * from a single barrel, with a parameter for any positional offset needed
 * between darts and a constant wait time between the first launch
 * and moving to the offset position for the second launch.
 * 
 * This command required a `StepperTurretSubsystem` for the dart pitch & yaw,
 * and an `AgitatorSubsystem` for the barrel sled.
 */
class LaunchDartComprisedCommand : public tap::control::ComprisedCommand
{
public:

    /**
     * @brief Construct a new Launch Darts Comprised Command object.
     * 
     * @param[in] drivers: A reference to the `drivers` singleton instance.
     * @param[in] turret: A reference to the `StepperTurretSubsystem`
     * for the pitch & yaw of the launcher.
     * @param[in] sled: A reference to the `AgitatorSubsystem`
     * for the barrel sled.
     * @param[in] config: The configuration parameters for this command,
     * as listed in the `LaunchDartComprisedCommand` struct.
     * @param[in] offsetConfig: The configuration for moving an offset after launching,
     * as listed in the `LaunchDartOffsetConifg` struct.
     * If the default value is passed, the command will end
     * immediately after launching the dart.
     */
    LaunchDartComprisedCommand(
        aruwsrc::Drivers *drivers,
        aruwsrc::control::turret::StepperTurretSubsystem& turret,
        aruwsrc::agitator::AgitatorSubsystem& sled,
        LaunchDartComprisedCommandConfig config,
        LaunchDartOffsetConfig offsetConfig=
        {
            .yawOffsetSteps = 0,
            .pitchOffsetSteps = 0,
            .timeDelayAfterMoving = 0.0f
        }
    );

    const char *getName() const override { return "launch darts"; }
    void initialize() override;
    void execute() override;
    void end(bool interrupted) override;
    bool isFinished() const override;

private:
    aruwsrc::Drivers* drivers;
    aruwsrc::control::turret::StepperTurretSubsystem& turret;
    aruwsrc::agitator::AgitatorSubsystem& sled;

    enum CommandState
    {
        LAUNCH_DART_ONE,
        MOVE_TO_OFFSET_POSITION,
        WAIT,
        FINISHED
    };
    CommandState state = LAUNCH_DART_ONE;

    tap::control::setpoint::MoveAbsoluteCommand moveSledToLaunchPosition;
    aruwsrc::control::turret::user::OffsetStepperMotorTurretControlCommand moveToOffsetPosition;

    float timeDelayAfterMoving;

    float timeInWaiting = 0.0f;
    float prevTime = 0.0f;
};
}

#endif  // LAUNCH_DART_COMPRISED_COMMAND_HPP_
