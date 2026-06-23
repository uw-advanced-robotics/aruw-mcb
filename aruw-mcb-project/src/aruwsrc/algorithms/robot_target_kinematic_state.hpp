/*
 * Copyright (c) 2025-2026 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef ROBOT_TARGET_KINEMATIC_STATE_HPP_
#define ROBOT_TARGET_KINEMATIC_STATE_HPP_

#include "tap/algorithms/ballistics.hpp"

namespace aruwsrc::algorithms
{
struct RobotTargetKinematicState : tap::algorithms::ballistics::SecondOrderKinematicState
{
    /**
     * Kinematic state of a single plate on an enemy robot
     *
     * @param position Position of the target robot center
     * @param velocity Velocity of the target robot center
     * @param acceleration Acceleration of the target robot center
     * @param radius Radius of the target robot
     * @param theta Angular position of the robot's plate in world frame
     * @param omega Angular velocity of the target robot
     */
    inline RobotTargetKinematicState(
        modm::Vector3f position,
        modm::Vector3f velocity,
        modm::Vector3f acceleration,
        float radius,
        float theta,
        float omega)
        : tap::algorithms::ballistics::SecondOrderKinematicState(position, velocity, acceleration),
          position(position),
          velocity(velocity),
          acceleration(acceleration),
          radius(radius),
          theta(theta),
          omega(omega)
    {
    }
    modm::Vector3f position;      // m
    modm::Vector3f velocity;      // m/s
    modm::Vector3f acceleration;  // m/s^2

    // rotation about center
    float radius{0};  // m
    float theta{0};   // rad
    float omega{0};   // rad/s

    /**
     * @param[in] dt: The amount of time to project the state forward.
     *
     * @return The future 3D position of the target plate on the robot using a quadratic (constant
     * acceleration) model for the center and linear (constant angular velocity) model for angle
     * about the center
     */
    inline modm::Vector3f projectForward(float dt) const override
    {
        float rxf = radius * cos(theta + omega * dt);
        float ryf = radius * sin(theta + omega * dt);
        return modm::Vector3f(
            quadraticKinematicProjection(dt, position.x, velocity.x, acceleration.x) + rxf,
            quadraticKinematicProjection(dt, position.y, velocity.y, acceleration.y) + ryf,
            quadraticKinematicProjection(dt, position.z, velocity.z, acceleration.z));
    }
};

}  // namespace aruwsrc::algorithms

#endif  // ROBOT_TARGET_KINEMATIC_STATE_HPP_