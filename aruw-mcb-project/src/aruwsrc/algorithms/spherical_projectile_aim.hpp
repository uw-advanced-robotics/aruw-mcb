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

#ifndef SPHERICAL_PROJECTILE_AIM_HPP_
#define SPHERICAL_PROJECTILE_AIM_HPP_

#include <cmath>
#include <cstdint>
#include <optional>

#include "tap/algorithms/ballistics.hpp"
#include "tap/algorithms/math_user_utils.hpp"

#include "modm/math/geometry/vector.hpp"

namespace aruwsrc::algorithms
{
struct SphereProjectileModel
{
    float diameterMeters;
    float massKg;
    float airDensityKgPerM3;
    float dynamicViscosityPas;

    float crossSectionalAreaMeters2() const
    {
        const float radiusMeters = diameterMeters / 2.0f;
        return M_PI * radiusMeters * radiusMeters;
    }

    float reynoldsNumber(float speedMetersPerSecond) const
    {
        if (speedMetersPerSecond <= 0.0f || dynamicViscosityPas <= 0.0f)
        {
            return 0.0f;
        }

        return airDensityKgPerM3 * speedMetersPerSecond * diameterMeters / dynamicViscosityPas;
    }

    float dragCoefficient(float speedMetersPerSecond) const
    {
        const float reynoldsNumber = this->reynoldsNumber(speedMetersPerSecond);
        if (reynoldsNumber <= 0.0f)
        {
            return 0.0f;
        }

        return 24.0f / reynoldsNumber +
               (2.6f * (reynoldsNumber / 5.0f)) / (1.0f + powf(reynoldsNumber / 5.0f, 1.52f)) +
               (0.411f * powf(reynoldsNumber / 263000.0f, -7.94f)) /
                   (1.0f + powf(reynoldsNumber / 263000.0f, -8.0f)) +
               powf(reynoldsNumber, 0.8f) / 461000.0f;
    }

    float dragAccelerationScale(float speedMetersPerSecond) const
    {
        return 0.5f * airDensityKgPerM3 * dragCoefficient(speedMetersPerSecond) *
               crossSectionalAreaMeters2() / massKg;
    }
};

static constexpr SphereProjectileModel ROBO_MASTER_17MM_SPHERE = {
    .diameterMeters = 0.017f,
    .massKg = 0.0032f,
    .airDensityKgPerM3 = 1.225f,
    .dynamicViscosityPas = 1.81e-5f,
};

struct SphereDragSolverConfig
{
    float maxFlightTimeSeconds = 1.2f;
    float minPitchRadians = -M_PI * 80.0f / 180.0f;
    float maxPitchRadians = M_PI * 45.0f / 180.0f;
    uint8_t correctionIterations = 2;
};

static constexpr SphereDragSolverConfig DEFAULT_SPHERE_DRAG_SOLVER_CONFIG = {};

struct SphereDragAimSolution
{
    float pitchAngle;
    float yawAngle;
    float timeOfFlight;
    float distance;
};

inline bool finiteAndPositive(float value) { return isfinite(value) && value > 0.0f; }

inline std::optional<float> estimateHorizontalDragTimeOfFlight(
    float horizontalDistance,
    float horizontalVelocity,
    float dragAccelerationScale,
    float referenceSpeed)
{
    if (!finiteAndPositive(horizontalDistance) || !finiteAndPositive(horizontalVelocity))
    {
        return std::nullopt;
    }

    const float dragRate = dragAccelerationScale * referenceSpeed;
    if (dragRate <= 0.0f)
    {
        return horizontalDistance / horizontalVelocity;
    }

    const float remainingVelocityRatio = 1.0f - dragRate * horizontalDistance / horizontalVelocity;
    if (remainingVelocityRatio <= 0.0f)
    {
        return std::nullopt;
    }

    return -logf(remainingVelocityRatio) / dragRate;
}

inline std::optional<float> estimatePitchForLinearVerticalDrag(
    float targetHeight,
    float launchSpeed,
    float timeOfFlight,
    float dragAccelerationScale,
    float referenceSpeed)
{
    if (!finiteAndPositive(launchSpeed) || !finiteAndPositive(timeOfFlight))
    {
        return std::nullopt;
    }

    const float dragRate = dragAccelerationScale * referenceSpeed;
    float initialVerticalVelocity = 0.0f;
    if (dragRate <= 0.0f)
    {
        initialVerticalVelocity = (targetHeight + 0.5f * tap::algorithms::ACCELERATION_GRAVITY *
                                                      timeOfFlight * timeOfFlight) /
                                  timeOfFlight;
    }
    else
    {
        const float oneMinusExp = 1.0f - expf(-dragRate * timeOfFlight);
        if (oneMinusExp <= 0.0f)
        {
            return std::nullopt;
        }

        initialVerticalVelocity =
            (targetHeight + tap::algorithms::ACCELERATION_GRAVITY * timeOfFlight / dragRate) *
                dragRate / oneMinusExp -
            tap::algorithms::ACCELERATION_GRAVITY / dragRate;
    }

    const float sinPitch = -initialVerticalVelocity / launchSpeed;
    if (!isfinite(sinPitch) || sinPitch < -1.0f || sinPitch > 1.0f)
    {
        return std::nullopt;
    }

    return asinf(sinPitch);
}

inline std::optional<SphereDragAimSolution> solveStationaryTargetWithSphereDrag(
    const modm::Vector3f &targetPosition,
    float launchSpeed,
    float pitchAxisOffset = 0.0f,
    const SphereProjectileModel &projectileModel = ROBO_MASTER_17MM_SPHERE,
    const SphereDragSolverConfig &solverConfig = DEFAULT_SPHERE_DRAG_SOLVER_CONFIG)
{
    const float horizontalDistance = hypotf(targetPosition.x, targetPosition.y) + pitchAxisOffset;
    if (!finiteAndPositive(horizontalDistance) || !finiteAndPositive(launchSpeed))
    {
        return std::nullopt;
    }

    float vacuumTimeOfFlight = 0.0f;
    float vacuumPitch = 0.0f;
    if (!tap::algorithms::ballistics::computeTravelTime(
            targetPosition,
            launchSpeed,
            &vacuumTimeOfFlight,
            &vacuumPitch,
            pitchAxisOffset))
    {
        return std::nullopt;
    }

    const float dragAccelerationScale = projectileModel.dragAccelerationScale(launchSpeed);
    float pitchAngle = tap::algorithms::limitVal(
        vacuumPitch,
        solverConfig.minPitchRadians,
        solverConfig.maxPitchRadians);
    float timeOfFlight = vacuumTimeOfFlight;
    for (uint8_t i = 0; i < solverConfig.correctionIterations; i++)
    {
        const float horizontalVelocity = launchSpeed * cosf(pitchAngle);
        auto dragTimeOfFlight = estimateHorizontalDragTimeOfFlight(
            horizontalDistance,
            horizontalVelocity,
            dragAccelerationScale,
            launchSpeed);
        if (!dragTimeOfFlight.has_value() || *dragTimeOfFlight <= 0.0f ||
            *dragTimeOfFlight > solverConfig.maxFlightTimeSeconds)
        {
            return std::nullopt;
        }

        timeOfFlight = *dragTimeOfFlight;
        auto estimatedPitch = estimatePitchForLinearVerticalDrag(
            targetPosition.z,
            launchSpeed,
            timeOfFlight,
            dragAccelerationScale,
            launchSpeed);
        if (!estimatedPitch.has_value())
        {
            return std::nullopt;
        }
        pitchAngle = tap::algorithms::limitVal(
            *estimatedPitch,
            solverConfig.minPitchRadians,
            solverConfig.maxPitchRadians);
    }

    return SphereDragAimSolution{
        .pitchAngle = pitchAngle,
        .yawAngle = atan2f(targetPosition.y, targetPosition.x),
        .timeOfFlight = timeOfFlight,
        .distance = targetPosition.getLength(),
    };
}

inline bool findTargetProjectileIntersectionWithSphereDrag(
    const tap::algorithms::ballistics::AbstractKinematicState &targetInitialState,
    float launchSpeed,
    uint8_t numIterations,
    float *turretPitch,
    float *turretYaw,
    float *projectedTravelTime,
    float pitchAxisOffset = 0.0f,
    float *distance = nullptr,
    const SphereProjectileModel &projectileModel = ROBO_MASTER_17MM_SPHERE,
    const SphereDragSolverConfig &solverConfig = DEFAULT_SPHERE_DRAG_SOLVER_CONFIG)
{
    if (turretPitch == nullptr || turretYaw == nullptr || projectedTravelTime == nullptr)
    {
        return false;
    }

    modm::Vector3f projectedTargetPosition = targetInitialState.projectForward(0.0f);
    if (projectedTargetPosition.x == 0.0f && projectedTargetPosition.y == 0.0f &&
        projectedTargetPosition.z == 0.0f)
    {
        return false;
    }

    std::optional<SphereDragAimSolution> solution = std::nullopt;

    for (uint8_t i = 0; i < numIterations; i++)
    {
        solution = solveStationaryTargetWithSphereDrag(
            projectedTargetPosition,
            launchSpeed,
            pitchAxisOffset,
            projectileModel,
            solverConfig);
        if (!solution.has_value())
        {
            return false;
        }

        projectedTargetPosition = targetInitialState.projectForward(solution->timeOfFlight);
    }

    *turretPitch = solution->pitchAngle;
    *turretYaw = atan2f(projectedTargetPosition.y, projectedTargetPosition.x);
    *projectedTravelTime = solution->timeOfFlight;
    if (distance != nullptr)
    {
        *distance = projectedTargetPosition.getLength();
    }

    return isfinite(*turretPitch) && isfinite(*turretYaw) && isfinite(*projectedTravelTime);
}
}  // namespace aruwsrc::algorithms

#endif  // SPHERICAL_PROJECTILE_AIM_HPP_
