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

#ifndef MOVE_UNJAM_REF_LIMITED_MULTI_SHOT_COMMAND_HPP_
#define MOVE_UNJAM_REF_LIMITED_MULTI_SHOT_COMMAND_HPP_

#include "tap/control/comprised_command.hpp"

#include "move_unjam_ref_limited_command.hpp"

namespace aruwsrc
{
class Drivers;
}

namespace aruwsrc::agitator
{
class AgitatorSubsystem;

class MoveUnjamRefLimitedMultiShotCommand : public tap::control::ComprisedCommand
{
public:
    enum ShooterState : uint8_t
    {
        SINGLE = 0,
        BURST,
        FULL_AUTO,
        NUM_SHOOTER_STATES,
    };

    MoveUnjamRefLimitedMultiShotCommand(
        aruwsrc::Drivers* drivers,
        AgitatorSubsystem* agitator17mm,
        float agitatorRotateAngle,
        float maxUnjamRotateAngle,
        uint32_t rotateTime,
        bool heatLimiting,
        float heatLimitBuffer,
        int burstNum);

    bool isReady() override;

    void initialize() override;

    void execute() override;

    bool isFinished() const override;

    void end(bool interrupted) override;

    const char* getName() const override { return "Move unjam multi shot"; }

    inline void setShooterState(ShooterState state) { shooterState = state; }

private:
    const int burstNum;

    ShooterState shooterState = ShooterState::SINGLE;

    int agitatorMovementsLeft = 0;

    MoveUnjamRefLimitedCommand moveUnjamRefLimitedCommand;
};  // class ShootFastComprisedCommand

}  // namespace aruwsrc::agitator

#endif  // MOVE_UNJAM_REF_LIMITED_MULTI_SHOT_COMMAND_HPP_
