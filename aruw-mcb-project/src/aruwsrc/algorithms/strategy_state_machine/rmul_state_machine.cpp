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

    if (!refSerial.getRefSerialReceivingData())
    {
        // Wait till we can get health data
        return;
    }

    uint16_t health = refSerial.getRobotData().currentHp;
    State prevState = state;

    switch (state)
    {
        case State::HEALING:
            // If we've healed enough, go back to attacking
            if (health >= ATTACKING_THRESHOLD && safeToAttack())
            {
                state = State::ATTACKING;
                updatePath(ATTACKING_PATH);
                pathTimeout.restart(PATH_LENGTH_MILLIS);
                patrolTimer.stop();
                patrolState = 0;
            }
            break;
        case State::ATTACKING:
            // If we're low on health, go to healing
            if (health < HEALING_THRESHOLD || !safeToAttack())
            {
                state = State::HEALING;
                updatePath(HEALING_PATH);
                pathTimeout.restart(PATH_LENGTH_MILLIS);
            }
            else if (pathTimeout.isExpired())
            {
                // Patrol
                if (patrolTimer.isStopped())
                {
                    patrolTimer.restart(PATROL_SEGMENT_LENGTH_MILLIS);
                }

                if (patrolTimer.execute())
                {
                    uint8_t newPatrolState = (patrolState + 1) % MODM_ARRAY_SIZE(PATROL_POINTS);
                    updatePath(
                        std::array<const Position, 2>(
                            {PATROL_POINTS[patrolState], PATROL_POINTS[newPatrolState]}));
                    patrolState = newPatrolState;
                }
            }
            break;
        default:
            break;
    }

    // if (state != prevState)
    // {
    //     updatePath();
    // }
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
        default:
            break;
    }
}

void RMULStateMachine::updatePath(const std::span<const Position> points)
{
    path.resetPath();
    for (const Position &point : points)
    {
        path.pushPoint(point);
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

bool RMULStateMachine::safeToAttack()
{
    bool projectilesSufficient =
        refSerial.getRobotData().turret.bulletsRemaining17 >= PROJECTILE_COUNT_THRESHOLD;
    bool visionOnline = visionCoprocessor.isCvOnline();
    return projectilesSufficient && visionOnline;
}

}  // namespace aruwsrc::algorithms::strategy_state_machine
