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

#ifndef BALLISTICS_SOLVER_INTERFACE_HPP_
#define BALLISTICS_SOLVER_INTERFACE_HPP_

#include <cmath>
#include <cstdint>
#include <optional>

namespace aruwsrc::algorithms
{
/**
 * Common interface for CV ballistics solvers
 */
class BallisticsSolverInterface
{
public:
    struct BallisticsSolution
    {
        float pitchAngle;
        float yawAngle;
        float distance;
        float timeOfFlight;
    };

    static constexpr float NUM_FORWARD_KINEMATIC_PROJECTIONS = 3;

    static constexpr float PLATE_WIDTH = 0.135f;
    static constexpr float PLATE_HEIGHT = 0.125f;

    static inline bool withinAimingTolerance(
        float yawAngleError,
        float pitchAngleError,
        float targetDistance)
    {
        if (targetDistance < 0)
        {
            return false;
        }

        return (abs(yawAngleError) < atan2f(PLATE_WIDTH, 2.0f * targetDistance)) &&
               (abs(pitchAngleError) < atan2f(PLATE_HEIGHT, 2.0f * targetDistance));
    }

    explicit BallisticsSolverInterface(uint8_t id) : turretID(id) {}

    virtual ~BallisticsSolverInterface() = default;

    virtual std::optional<BallisticsSolution> computeTurretAimAngles() = 0;

    const uint8_t turretID;
};
}  // namespace aruwsrc::algorithms

#endif  // BALLISTICS_SOLVER_INTERFACE_HPP_
