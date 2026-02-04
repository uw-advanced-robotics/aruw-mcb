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

#ifndef RMUL_STATE_MACHINE_HPP_
#define RMUL_STATE_MACHINE_HPP_

#include <array>

#include <span>

#include "tap/architecture/periodic_timer.hpp"
#include "tap/architecture/timeout.hpp"
#include "tap/communication/serial/ref_serial.hpp"

#include "aruwsrc/algorithms/auto_nav_path.hpp"
#include "aruwsrc/communication/serial/vision_coprocessor.hpp"
#include "aruwsrc/control/chassis/chassis_auto_nav_controller.hpp"

/**
 * Field:
 * Not to scale or accurate
 * +------------------------------------------+
 * |                          | Blue Loading  |
 * |                          |_______________|
 * |                                          |
 * |                    ______________________|
 * |                    |                     |
 * |                    |  Elevated platform  |
 * |          --------------------------      |
 * |                                   |      |
 * |                                   |______|
 * |                                          |
 * |              ------------                |
 * |             |            |               |
 * |-------      |   Capture  |        -------|
 * |             |   Point    |               |
 * |  X           ------------                |
 * | (POINT 4)         X                      |
 * |______            (POINT 3)               |
 * |      |                                   |
 * |      |                                   |
 * |      ------------------------     X      |
 * |Elevated platform        |     (POINT 2)  |
 * |_________________________|                |
 * |                                          |
 * |___________             X                 |
 * | Red      |     (POINT 1)                 |
 * | Loading  |                               |
 * +------------------------------------------+
 *
 */

namespace aruwsrc::algorithms::strategy_state_machine
{
using namespace tap::communication::serial;
using namespace aruwsrc::control::chassis;
using namespace aruwsrc::algorithms;
using namespace tap::algorithms::transforms;
class RMULStateMachine
{
public:
    RMULStateMachine(
        RefSerial& refSerial,
        aruwsrc::communication::serial::VisionCoprocessor& visionCoprocessor)
        : refSerial(refSerial),
          visionCoprocessor(visionCoprocessor)
    {
    }

    void updateState();

    void attachAutoNavController(ChassisAutoNavController* autoNavController);

private:
    RefSerial& refSerial;
    aruwsrc::communication::serial::VisionCoprocessor& visionCoprocessor;
    ChassisAutoNavController* autoNavController;

    AutoNavPath path;

    tap::arch::MilliTimeout pathTimeout;
    tap::arch::PeriodicMilliTimer patrolTimer;

    enum State
    {
        HEALING,
        ATTACKING,
    };
    State state = State::ATTACKING;
    uint8_t patrolState{0};

    void updatePath(const std::span<const Position> points);

    bool safeToAttack();

    // Threshold at which the robot goes to heal due to low health
    int HEALING_THRESHOLD = 200;

    // Threshold at which the robot goes back to fight having healed
    int ATTACKING_THRESHOLD = 375;

    int PROJECTILE_COUNT_THRESHOLD = 100;  // Minimum number of projectiles to attack

    // Speed at which the robot moves when healing, in m/s
    float SPEED = 5.0f;

    const Position RESUPPLY_ZONE = Position(0.75, 7, 0);
    const Position POINT_1 = Position(1.2, 2.1, 0);   // BOTTOM_MIDDLE
    const Position POINT_2 = Position(3.5, 1.5, 0);   // MIDDLE_RIGHT
    const Position POINT_3 = Position(5.25, 1.1, 0);  // RIGHT SIDE_WALL
    const Position PATROL_POINTS[2]{
        POINT_3,
        POINT_3 - Vector(3, 0.1, 0)};  // BIT BEHIND RIGHT SIDE_WALL

    const std::array<const Position, 4> ATTACKING_PATH = {RESUPPLY_ZONE, POINT_1, POINT_2, POINT_3};

    const std::array<const Position, 4> HEALING_PATH = {POINT_3, POINT_2, POINT_1, RESUPPLY_ZONE};

    const std::array<const Position, 2> ENGINEER_PATH = {POINT_1, POINT_2};

    static constexpr uint16_t PATH_LENGTH_MILLIS = 11000;
    static constexpr uint16_t PATROL_SEGMENT_LENGTH_MILLIS = 5000;
};
}  // namespace aruwsrc::algorithms::strategy_state_machine

#endif  // RMUL_STATE_MACHINE_HPP_
