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

#ifndef LAUNCH_DARTS_COMPRISED_COMMAND_HPP_
#define LAUNCH_DARTS_COMPRISED_COMMAND_HPP_

#include "aruwsrc/drivers.hpp"
#include "tap/control/comprised_command.hpp"
#include "tap/control/setpoint/commands/move_absolute_command.hpp"

#include "aruwsrc/control/turret/stepper_turret_subsystem.hpp"
#include "aruwsrc/control/turret/user/stepper_motor_turret_control_command.hpp"
#include "aruwsrc/control/agitator/agitator_subsystem.hpp"

namespace aruwsrc::control::dart 
{
struct LaunchDartsComprisedCommandConfig
{
    float dartOneAgitatorSetpoint;
    float dartTwoAgitatorSetpoint;
    float agitatorAngularSpeed;
    float agitatorSetpointTolerance;
    float yawOffsetSteps;
    float pitchOffsetSteps;
};

class LaunchDartsComprisedCommand : public tap::control::ComprisedCommand
{
public:
    LaunchDartsComprisedCommand(
        aruwsrc::Drivers *drivers,
        aruwsrc::control::turret::StepperTurretSubsystem& turret,
        aruwsrc::agitator::AgitatorSubsystem& sled,
        LaunchDartsComprisedCommandConfig config
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
        WAIT,
        MOVE_TO_OFFSET_POSITION,
        LAUNCH_DART_TWO,
        FINISHED
    };
    CommandState state;

    tap::control::setpoint::MoveAbsoluteCommand moveSledToPositionOne, moveSledToPositionTwo;
    aruwsrc::control::turret::user::OffsetStepperMotorTurretControlCommand moveToOffsetPosition;
};
}

#endif  // LAUNCH_DARTS_COMPRISED_COMMAND_HPP_
