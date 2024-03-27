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

#ifndef CONSTANT_VELOCITY_AGITATOR_COMMAND_HPP_
#define CONSTANT_VELOCITY_AGITATOR_COMMAND_HPP_

#include <float.h>

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/control/setpoint/commands/move_integral_command.hpp"

namespace aruwsrc::control::agitator
{

/**
 * A command that aims to keep the agitator at a constant velocity. At the end of this command,
 * the agitator will move to a specified setpoint, as to give a consistent starting point for each
 * shot.
 *
 * Ends if the agitator is offline or jammed.
 */

class ConstantVelocityAgitatorCommand : public tap::control::setpoint::MoveIntegralCommand
{
public:
    ConstantVelocityAgitatorCommand(
        tap::control::setpoint::IntegrableSetpointSubsystem& integrableSetpointSubsystem,
        const Config& config)
        : tap::control::setpoint::MoveIntegralCommand(integrableSetpointSubsystem, config),
          subsystem(integrableSetpointSubsystem)
    {
    }

    const char* getName() const override { return "Constant velocity agitator command"; }

    void end(bool interrupted) override
    {
        if (useSingleShotMode || interrupted)
        {
            subsystem.setSetpoint(0);
        }
    };

    bool isFinished() const
    {
        // The subsystem is jammed or offline or it is within the setpoint tolerance
        return subsystem.isJammed() || !subsystem.isOnline() || (useSingleShotMode && targetIntegralReached());
    };

    void setConstantRotation(bool constantRotation)
    {
        useSingleShotMode = !constantRotation;
        if(useSingleShotMode){
            while(finalTargetIntegralSetpoint < subsystem.getCurrentValueIntegral()){
                finalTargetIntegralSetpoint += config.targetIntegralChange;
            }
        }
    }


    tap::control::setpoint::IntegrableSetpointSubsystem& subsystem;

    bool useSingleShotMode = true;
};

}  // namespace aruwsrc::control::agitator
#endif  // CONSTANT_VELOCITY_AGITATOR_COMMAND_HPP_
