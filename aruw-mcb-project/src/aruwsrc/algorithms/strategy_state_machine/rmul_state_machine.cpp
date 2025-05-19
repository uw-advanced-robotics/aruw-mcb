/*
 * Copyright (c) 2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "rmul_state_machine.hpp"

namespace aruwsrc::algorithms::strategy_state_machine
{
void RMULStateMachine::updateState()
{
    if (autoNavController == nullptr)
    {
        return;
    }

    uint16_t health = refSerial.getRobotData().currentHp;
    State prevState = state;

    switch (state)
    {
        case State::HEALING:
            // If we've healed enough, go back to attacking
            if (health >= ATTACKING_THRESHOLD)
            {
                state = State::ATTACKING;
            }
            break;
        case State::ATTACKING:
            // If we're low on health, go to healing
            if (health < HEALING_THRESHOLD)
            {
                state = State::HEALING;
            }
            break;
        case State::FIRST_PUSH:
            // If we're low on health, go to healing
            if (health < HEALING_THRESHOLD)
            {
                state = State::HEALING;
            }
            break;
        default:
            break;
    }

    if (state != prevState)
    {
        updatePath();
    }
}

void RMULStateMachine::updatePath()
{
    path.resetPath();

    switch (state)
    {
        case State::HEALING:
            for (auto &point : HEALING_PATH)
            {
                path.pushPoint(point);
            }
            break;
        case State::ATTACKING:
            for (auto &point : ATTACKING_PATH)
            {
                path.pushPoint(point);
            }
            break;
        case State::FIRST_PUSH:
            for (auto &point : FIRST_PUSH_PATH)
            {
                path.pushPoint(point);
            }
            break;
        default:
            break;
    }
}

void RMULStateMachine::attachAutoNavController(ChassisAutoNavController *autoNavController)
{
    this->autoNavController = autoNavController;
    this->autoNavController->attachPath(&path);
    this->autoNavController->setDesiredSpeed(SPEED);

    // Load initial path
    updatePath();
}

}  // namespace aruwsrc::algorithms::strategy_state_machine
