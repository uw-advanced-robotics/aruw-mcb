/*
 * Copyright (c) 2020-2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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
#ifndef TURRET_FEED_FORWARD_INTERFACE_HPP_
#define TURRET_FEED_FORWARD_INTERFACE_HPP_

#include <vector>

/**
 * @brief Abstract interface for SISO feedforward control calculation
 *
 * Derived classes should override the calculate() method to implement
 * their own feedforward logic.
 */
namespace aruwsrc::control::turret::algorithms
{
class TurretFeedforwardInterface
{
public:
    struct TurretFeedforwardState
    {
        float pitch;
        float yaw;
    };
    /**
     * @brief Calculates the feedforward control output.
     *
     * @param TurretFeedforwardState system state struct
     * @return float Feedforward control output (SISO)
     */
    virtual float calculateFeedforward(const TurretFeedforwardState state) const = 0;
};
};  // namespace aruwsrc::control::turret::algorithms

#endif  // FEED_FORWARD_INTERFACE_HPP_