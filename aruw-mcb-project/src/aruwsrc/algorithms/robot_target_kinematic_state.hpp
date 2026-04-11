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
     * @param[in] dt: The amount of time to project forward.
     * @param[in] s: The position of the object.
     * @param[in] v: The velocity of the object.
     * @param[in] a: The acceleration of the object.
     *
     * @return The future position of an object using a quadratic (constant acceleration) model.
     */
    inline static float quadraticKinematicProjection(float dt, float s, float v, float a)
    {
        return s + v * dt + 0.5f * a * dt * dt;
    }

    /**
     * @param[in] dt: The amount of time to project the state forward.
     *
     * @return The future 3D position of this object using a quadratic (constant acceleration)
     * model for the center and linear (constant angular velocity) model for angle about the
     * center
     */
    inline modm::Vector3f projectForward(float dt) const override
    {
        float rx = radius * cos(theta);
        float ry = radius * sin(theta);
        float rxf = radius * cos(theta + omega * dt);
        float ryf = radius * sin(theta + omega * dt);
        return modm::Vector3f(
            quadraticKinematicProjection(dt, position.x - rx, velocity.x, acceleration.x) + rxf,
            quadraticKinematicProjection(dt, position.y - ry, velocity.y, acceleration.y) + ryf,
            quadraticKinematicProjection(dt, position.z, velocity.z, acceleration.z));
    }
    /**
     * Compute the angular velocity of the target wrt a rotating frame who's x axis always faces
     * the target (e.g. the turret tracks the target robot center)
     *
     * $$ omega_{total} = omega_{robot} + \frac{(r \times v)_z}{|r|^2} $$
     *
     * @param robotPos Robot position relative to observer (turret, static frame)
     * @param robotVel Robot velocity relative to observer (turret, static frame)
     * @return Total angular velocity as seen from observer rotating frame (rad/s)
     */
    inline float computeOmegaTotal(const modm::Vector3f& robotPos, const modm::Vector3f& robotVel)
        const
    {
        // Compute cross product (r x v)_z component
        float crossProductZ = robotPos.x * robotVel.y - robotPos.y * robotVel.x;

        // Magnitude squared of r (in x-y plane)
        float rMagSquared = robotPos.x * robotPos.x + robotPos.y * robotPos.y;

        // Avoid division by zero
        if (rMagSquared < 1e-6f)
        {
            return omega;
        }

        float omegaFromTranslation = crossProductZ / rMagSquared;
        return omega + omegaFromTranslation;
    }

    /**
     * Determines the active plate index based on time of flight and total angular velocity.
     * Finds the plate with a valid future fire window whose center is closest to ToF.
     * @param omegaTotal Total angular velocity accounting for rotation and translation (rad/s)
     * @param timeOfFlight Time for projectile to reach target (s)
     * @param plateWidth Width of armor plate (m)
     * @param aimAngle Angle of our aim line (from turret to robot center) in world frame (rad)
     * @param currentTheta Angular position of plate 0 in world frame (rad)
     * @return Active plate index (0-3), where 0 is current closest plate
     */
    inline uint8_t determineActivePlate(
        float omegaTotal,
        float timeOfFlight,
        float plateWidth,
        float aimAngle,
        float currentTheta) const
    {
        // Minimum fire window duration to be considered valid (100ms)
        constexpr float MIN_FIRE_WINDOW_S = 0.1f;

        uint8_t bestPlate = 0;
        float bestTimeDifference = 1e9f;  // Large initial value
        bool foundValidPlate = false;

        // Check each plate (0-3) to find one with a valid fire window
        for (int i = 0; i < 4; i++)
        {
            // Calculate actual angular position of plate i
            float plateAngle = currentTheta + i * M_PI_2;

            // Angular distance from plate to aim line
            float angularOffset = plateAngle - aimAngle;

            // Normalize to [-π, π]
            while (angularOffset > M_PI) angularOffset -= 2.0f * M_PI;
            while (angularOffset < -M_PI) angularOffset += 2.0f * M_PI;

            // Calculate time for plate center to reach aim line
            float timeToCenterEdge;
            if (omegaTotal > 0)
            {
                // Counterclockwise rotation
                if (angularOffset < 0)
                {
                    // Plate is behind, add full rotation
                    angularOffset += 2.0f * M_PI;
                }
                timeToCenterEdge = angularOffset / omegaTotal;
            }
            else
            {
                // Clockwise rotation
                if (angularOffset > 0)
                {
                    // Plate is ahead, subtract full rotation
                    angularOffset -= 2.0f * M_PI;
                }
                timeToCenterEdge =
                    angularOffset / omegaTotal;  // angularOffset negative, omega negative
            }

            // Calculate when we'd need to fire to hit this plate
            // We need the far edge time to determine if the fire window is still open
            float plateAngularWidth = plateWidth / radius;
            float halfWidthTime = (plateAngularWidth / 2.0f) / fabsf(omegaTotal);
            float timeToFarEdge = timeToCenterEdge + halfWidthTime;
            float fireWindowEnd = timeToFarEdge - timeOfFlight;

            // Only consider plates whose fire window hasn't closed yet
            // (with minimum window requirement)
            if (fireWindowEnd >= MIN_FIRE_WINDOW_S)
            {
                float timeDifference = fabsf(timeToCenterEdge - timeOfFlight);
                if (!foundValidPlate || timeDifference < bestTimeDifference)
                {
                    bestPlate = i;
                    bestTimeDifference = timeDifference;
                    foundValidPlate = true;
                }
            }
        }

        return bestPlate;
    }
};

}  // namespace aruwsrc::algorithms

#endif  // ROBOT_TARGET_KINEMATIC_STATE_HPP_