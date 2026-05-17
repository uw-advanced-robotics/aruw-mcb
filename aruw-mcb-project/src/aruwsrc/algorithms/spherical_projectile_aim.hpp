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

    float dragCoefficient(float speedMetersPerSecond) const
    {
        if (speedMetersPerSecond <= 0.0f || dynamicViscosityPas <= 0.0f)
        {
            return 0.0f;
        }

        const float reynoldsNumber =
            airDensityKgPerM3 * speedMetersPerSecond * diameterMeters / dynamicViscosityPas;
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
    float timestepSeconds = 0.01f;
    float maxFlightTimeSeconds = 1.2f;
    float minPitchRadians = -M_PI * 80.0f / 180.0f;
    float maxPitchRadians = M_PI * 45.0f / 180.0f;
    float localPitchSearchRadians = M_PI * 25.0f / 180.0f;
    uint8_t pitchBisectionIterations = 8;
};

static constexpr SphereDragSolverConfig DEFAULT_SPHERE_DRAG_SOLVER_CONFIG = {};

struct SphereDragAimSolution
{
    float pitchAngle;
    float yawAngle;
    float timeOfFlight;
    float distance;
};

struct SphereProjectileState
{
    float horizontalPosition;
    float verticalPosition;
    float horizontalVelocity;
    float verticalVelocity;
};

struct SphereDragIntersection
{
    float verticalError;
    float timeOfFlight;
};

inline bool finiteAndPositive(float value) { return isfinite(value) && value > 0.0f; }

inline SphereProjectileState computeProjectileStateDerivative(
    const SphereProjectileState &state,
    float dragAccelerationScale)
{
    const float speed = sqrtf(
        state.horizontalVelocity * state.horizontalVelocity +
        state.verticalVelocity * state.verticalVelocity);

    return SphereProjectileState{
        .horizontalPosition = state.horizontalVelocity,
        .verticalPosition = state.verticalVelocity,
        .horizontalVelocity = -dragAccelerationScale * speed * state.horizontalVelocity,
        .verticalVelocity = -tap::algorithms::ACCELERATION_GRAVITY -
                            dragAccelerationScale * speed * state.verticalVelocity,
    };
}

inline SphereProjectileState addScaledState(
    const SphereProjectileState &state,
    const SphereProjectileState &delta,
    float scale)
{
    return SphereProjectileState{
        .horizontalPosition = state.horizontalPosition + delta.horizontalPosition * scale,
        .verticalPosition = state.verticalPosition + delta.verticalPosition * scale,
        .horizontalVelocity = state.horizontalVelocity + delta.horizontalVelocity * scale,
        .verticalVelocity = state.verticalVelocity + delta.verticalVelocity * scale,
    };
}

inline SphereProjectileState rungeKuttaIntegrateProjectileState(
    const SphereProjectileState &state,
    float timestepSeconds,
    float dragAccelerationScale)
{
    const SphereProjectileState k1 = computeProjectileStateDerivative(state, dragAccelerationScale);
    const SphereProjectileState k2 = computeProjectileStateDerivative(
        addScaledState(state, k1, timestepSeconds * 0.5f),
        dragAccelerationScale);
    const SphereProjectileState k3 = computeProjectileStateDerivative(
        addScaledState(state, k2, timestepSeconds * 0.5f),
        dragAccelerationScale);
    const SphereProjectileState k4 = computeProjectileStateDerivative(
        addScaledState(state, k3, timestepSeconds),
        dragAccelerationScale);

    return SphereProjectileState{
        .horizontalPosition =
            state.horizontalPosition + timestepSeconds *
                                           (k1.horizontalPosition + 2.0f * k2.horizontalPosition +
                                            2.0f * k3.horizontalPosition + k4.horizontalPosition) /
                                           6.0f,
        .verticalPosition =
            state.verticalPosition + timestepSeconds *
                                         (k1.verticalPosition + 2.0f * k2.verticalPosition +
                                          2.0f * k3.verticalPosition + k4.verticalPosition) /
                                         6.0f,
        .horizontalVelocity =
            state.horizontalVelocity + timestepSeconds *
                                           (k1.horizontalVelocity + 2.0f * k2.horizontalVelocity +
                                            2.0f * k3.horizontalVelocity + k4.horizontalVelocity) /
                                           6.0f,
        .verticalVelocity =
            state.verticalVelocity + timestepSeconds *
                                         (k1.verticalVelocity + 2.0f * k2.verticalVelocity +
                                          2.0f * k3.verticalVelocity + k4.verticalVelocity) /
                                         6.0f,
    };
}

inline std::optional<SphereDragIntersection> simulateSphereProjectileToRange(
    float horizontalDistance,
    float targetHeight,
    float launchSpeed,
    float pitchAngle,
    const SphereProjectileModel &projectileModel = ROBO_MASTER_17MM_SPHERE,
    const SphereDragSolverConfig &solverConfig = DEFAULT_SPHERE_DRAG_SOLVER_CONFIG)
{
    if (!finiteAndPositive(horizontalDistance) || !finiteAndPositive(launchSpeed) ||
        !finiteAndPositive(solverConfig.timestepSeconds) ||
        !finiteAndPositive(solverConfig.maxFlightTimeSeconds))
    {
        return std::nullopt;
    }

    SphereProjectileState projectileState{
        .horizontalPosition = 0.0f,
        .verticalPosition = 0.0f,
        .horizontalVelocity = launchSpeed * cosf(pitchAngle),
        .verticalVelocity = -launchSpeed * sinf(pitchAngle),
    };

    if (projectileState.horizontalVelocity <= 0.0f)
    {
        return std::nullopt;
    }

    const float dragAccelerationScale = projectileModel.dragAccelerationScale(launchSpeed);
    float previousTimeSeconds = 0.0f;
    for (float timeSeconds = solverConfig.timestepSeconds;
         timeSeconds <= solverConfig.maxFlightTimeSeconds;
         timeSeconds += solverConfig.timestepSeconds)
    {
        const SphereProjectileState previousState = projectileState;
        projectileState = rungeKuttaIntegrateProjectileState(
            projectileState,
            solverConfig.timestepSeconds,
            dragAccelerationScale);

        if (projectileState.horizontalPosition >= horizontalDistance)
        {
            const float horizontalDelta =
                projectileState.horizontalPosition - previousState.horizontalPosition;
            const float interpolationRatio =
                horizontalDelta > 0.0f
                    ? (horizontalDistance - previousState.horizontalPosition) / horizontalDelta
                    : 1.0f;

            return SphereDragIntersection{
                .verticalError = previousState.verticalPosition +
                                 interpolationRatio * (projectileState.verticalPosition -
                                                       previousState.verticalPosition) -
                                 targetHeight,
                .timeOfFlight =
                    previousTimeSeconds + interpolationRatio * solverConfig.timestepSeconds,
            };
        }

        if (projectileState.horizontalVelocity <= 0.0f)
        {
            return std::nullopt;
        }

        previousTimeSeconds = timeSeconds;
    }

    return std::nullopt;
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

    const float centerPitch = tap::algorithms::limitVal(
        vacuumPitch,
        solverConfig.minPitchRadians,
        solverConfig.maxPitchRadians);
    float lowPitch = tap::algorithms::limitVal(
        centerPitch - solverConfig.localPitchSearchRadians,
        solverConfig.minPitchRadians,
        solverConfig.maxPitchRadians);
    float highPitch = tap::algorithms::limitVal(
        centerPitch + solverConfig.localPitchSearchRadians,
        solverConfig.minPitchRadians,
        solverConfig.maxPitchRadians);

    auto lowIntersection = simulateSphereProjectileToRange(
        horizontalDistance,
        targetPosition.z,
        launchSpeed,
        lowPitch,
        projectileModel,
        solverConfig);
    auto highIntersection = simulateSphereProjectileToRange(
        horizontalDistance,
        targetPosition.z,
        launchSpeed,
        highPitch,
        projectileModel,
        solverConfig);
    if (!lowIntersection.has_value() || !highIntersection.has_value() ||
        lowIntersection->verticalError * highIntersection->verticalError > 0.0f)
    {
        return std::nullopt;
    }

    SphereDragIntersection rootIntersection = highIntersection.value();
    float lowError = lowIntersection->verticalError;
    for (uint8_t i = 0; i < solverConfig.pitchBisectionIterations; i++)
    {
        const float midPitch = (lowPitch + highPitch) * 0.5f;
        auto midIntersection = simulateSphereProjectileToRange(
            horizontalDistance,
            targetPosition.z,
            launchSpeed,
            midPitch,
            projectileModel,
            solverConfig);

        if (!midIntersection.has_value())
        {
            highPitch = midPitch;
            continue;
        }

        rootIntersection = midIntersection.value();
        if (lowError * midIntersection->verticalError <= 0.0f)
        {
            highPitch = midPitch;
        }
        else
        {
            lowPitch = midPitch;
            lowError = midIntersection->verticalError;
        }
    }

    return SphereDragAimSolution{
        .pitchAngle = (lowPitch + highPitch) * 0.5f,
        .yawAngle = atan2f(targetPosition.y, targetPosition.x),
        .timeOfFlight = rootIntersection.timeOfFlight,
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
