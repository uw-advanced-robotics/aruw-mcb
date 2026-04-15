#ifndef SPHERICAL_PROJECTILE_AIM_HPP_
#define SPHERICAL_PROJECTILE_AIM_HPP_

#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>

#include "modm/math/geometry/angle.hpp"
#include "tap/algorithms/ballistics.hpp"

namespace aruwsrc::algorithms
{
/**
 * Sphere projectile model and drag solver assumptions:
 *
 * 1. NASA-CR-1392, "Subsonic sphere drag measurements at intermediate Reynolds numbers"
 *    (Nicholls and Sivier, 1969), for the general Reynolds-dependent behavior of smooth spheres:
 *    https://ntrs.nasa.gov/citations/19690021231
 *
 * 2. Morrison, F. A. (2013), "Data Correlation for Sphere Drag Coefficient for Fluid Mechanics
 *    and Multiphase Flow Students", which provides the Cd(Re) correlation implemented below:
 *    https://www.researchgate.net/publication/257724098_Data_Correlation_for_Sphere_Drag_Coefficient_for_Fluid_Mechanics_and_Multiphase_Flow_Students
 */
struct SphereProjectileModel
{
    float diameterMeters;
    float massKg;
    float airDensityKgPerM3;
    float dynamicViscosityPas;

    constexpr float radiusMeters() const { return diameterMeters / 2.0f; }

    constexpr float crossSectionalAreaMeters2() const
    {
        return M_PI * radiusMeters() * radiusMeters();
    }

    constexpr float dragAccelerationScale(float dragCoefficient) const
    {
        return 0.5f * airDensityKgPerM3 * dragCoefficient * crossSectionalAreaMeters2() / massKg;
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
    float timestepSeconds;
    float maxSimulationTimeSeconds;
    float angleToleranceRadians;
    float heightToleranceMeters;
    uint8_t movingTargetIterations;
    uint8_t pitchSolverIterations;
};

static constexpr SphereDragSolverConfig DEFAULT_SPHERE_DRAG_SOLVER_CONFIG = {
    .timestepSeconds = 0.002f,
    .maxSimulationTimeSeconds = 2.0f,
    .angleToleranceRadians = modm::toRadian(0.01f),
    .heightToleranceMeters = 0.01f,
    .movingTargetIterations = 2,
    .pitchSolverIterations = 6,
};

struct DragIntersection
{
    float verticalError;
    float timeOfFlight;
};

struct SphereProjectileState
{
    float horizontalPosition;
    float verticalPosition;
    float horizontalVelocity;
    float verticalVelocity;
};

static constexpr float MIN_PROJECTILE_PITCH_RADIANS = -M_PI_2;
static constexpr float MAX_PROJECTILE_PITCH_RADIANS = M_PI_2;
static constexpr float PITCH_SOLVER_DELTA_RADIANS = modm::toRadian(1.0f);
static constexpr float MAX_LOCAL_PITCH_CORRECTION_RADIANS = modm::toRadian(8.0f);

inline SphereProjectileState computeProjectileStateDerivative(
    const SphereProjectileState &state,
    const SphereProjectileModel &projectileModel)
{
    const float speed =
        sqrtf(state.horizontalVelocity * state.horizontalVelocity +
              state.verticalVelocity * state.verticalVelocity);

    constexpr float GRAVITY_ACCELERATION_METERS_PER_SECOND2 = 9.81f;
    const float reynoldsNumber =
        projectileModel.dynamicViscosityPas > 0.0f
            ? projectileModel.airDensityKgPerM3 * speed * projectileModel.diameterMeters /
                  projectileModel.dynamicViscosityPas
            : 0.0f;

    float dragCoefficient = 0.0f;
    if (reynoldsNumber > 0.0f)
    {
        // Smooth-sphere Cd(Re) correlation from Morrison (2013), consistent with the
        // Reynolds-dependent sphere behavior reported in NASA-CR-1392.
        dragCoefficient =
            24.0f / reynoldsNumber +
            (2.6f * (reynoldsNumber / 5.0f)) / (1.0f + powf(reynoldsNumber / 5.0f, 1.52f)) +
            (0.411f * powf(reynoldsNumber / 263000.0f, -7.94f)) /
                (1.0f + powf(reynoldsNumber / 263000.0f, -8.0f)) +
            powf(reynoldsNumber, 0.8f) / 461000.0f;
    }

    const float dragAccelerationScale = projectileModel.dragAccelerationScale(dragCoefficient);

    return SphereProjectileState{
        .horizontalPosition = state.horizontalVelocity,
        .verticalPosition = state.verticalVelocity,
        .horizontalVelocity = -dragAccelerationScale * speed * state.horizontalVelocity,
        .verticalVelocity =
            -GRAVITY_ACCELERATION_METERS_PER_SECOND2 -
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
    const SphereProjectileModel &projectileModel)
{
    const SphereProjectileState k1 = computeProjectileStateDerivative(state, projectileModel);
    const SphereProjectileState k2 = computeProjectileStateDerivative(
        addScaledState(state, k1, timestepSeconds * 0.5f),
        projectileModel);
    const SphereProjectileState k3 = computeProjectileStateDerivative(
        addScaledState(state, k2, timestepSeconds * 0.5f),
        projectileModel);
    const SphereProjectileState k4 = computeProjectileStateDerivative(
        addScaledState(state, k3, timestepSeconds),
        projectileModel);

    SphereProjectileState nextState = state;
    nextState.horizontalPosition +=
        timestepSeconds *
        (k1.horizontalPosition + 2.0f * k2.horizontalPosition + 2.0f * k3.horizontalPosition +
         k4.horizontalPosition) /
        6.0f;
    nextState.verticalPosition +=
        timestepSeconds *
        (k1.verticalPosition + 2.0f * k2.verticalPosition + 2.0f * k3.verticalPosition +
         k4.verticalPosition) /
        6.0f;
    nextState.horizontalVelocity +=
        timestepSeconds *
        (k1.horizontalVelocity + 2.0f * k2.horizontalVelocity + 2.0f * k3.horizontalVelocity +
         k4.horizontalVelocity) /
        6.0f;
    nextState.verticalVelocity +=
        timestepSeconds *
        (k1.verticalVelocity + 2.0f * k2.verticalVelocity + 2.0f * k3.verticalVelocity +
         k4.verticalVelocity) /
        6.0f;

    return nextState;
}

inline std::optional<DragIntersection> simulateSphereDragIntersection(
    float horizontalDistance,
    float targetHeight,
    float launchSpeed,
    float pitchAngle,
    float pitchAxisOffset = 0.0f,
    const SphereProjectileModel &projectileModel = ROBO_MASTER_17MM_SPHERE,
    const SphereDragSolverConfig &solverConfig = DEFAULT_SPHERE_DRAG_SOLVER_CONFIG)
{
    if (horizontalDistance <= 0.0f || launchSpeed <= 0.0f)
    {
        return std::nullopt;
    }

    horizontalDistance += pitchAxisOffset;
    if (horizontalDistance <= 0.0f)
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

    float previousTimeSeconds = 0.0f;

    for (float timeSeconds = solverConfig.timestepSeconds;
         timeSeconds <= solverConfig.maxSimulationTimeSeconds;
         timeSeconds += solverConfig.timestepSeconds)
    {
        const SphereProjectileState previousState = projectileState;
        projectileState = rungeKuttaIntegrateProjectileState(
            projectileState,
            solverConfig.timestepSeconds,
            projectileModel);

        if (projectileState.horizontalPosition >= horizontalDistance)
        {
            const float horizontalDelta =
                projectileState.horizontalPosition - previousState.horizontalPosition;
            const float interpolationRatio =
                (horizontalDelta > 0.0f)
                    ? (horizontalDistance - previousState.horizontalPosition) / horizontalDelta
                    : 1.0f;

            return DragIntersection{
                .verticalError =
                    previousState.verticalPosition +
                    interpolationRatio *
                        (projectileState.verticalPosition - previousState.verticalPosition) -
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

inline bool solveSphereDragPitch(
    float horizontalDistance,
    float targetHeight,
    float launchSpeed,
    float initialPitchGuess,
    float *pitchAngle,
    float *timeOfFlight,
    float pitchAxisOffset = 0.0f,
    const SphereProjectileModel &projectileModel = ROBO_MASTER_17MM_SPHERE,
    const SphereDragSolverConfig &solverConfig = DEFAULT_SPHERE_DRAG_SOLVER_CONFIG)
{
    if (pitchAngle == nullptr || timeOfFlight == nullptr)
    {
        return false;
    }

    const float minPitchSearch =
        fmaxf(MIN_PROJECTILE_PITCH_RADIANS, initialPitchGuess - MAX_LOCAL_PITCH_CORRECTION_RADIANS);
    const float maxPitchSearch =
        fminf(MAX_PROJECTILE_PITCH_RADIANS, initialPitchGuess + MAX_LOCAL_PITCH_CORRECTION_RADIANS);

    auto evaluatePitch = [&](float angle, DragIntersection *intersection) -> bool
    {
        if (angle < minPitchSearch || angle > maxPitchSearch)
        {
            return false;
        }

        const std::optional<DragIntersection> evaluation = simulateSphereDragIntersection(
            horizontalDistance,
            targetHeight,
            launchSpeed,
            angle,
            pitchAxisOffset,
            projectileModel,
            solverConfig);

        if (!evaluation.has_value())
        {
            return false;
        }

        *intersection = evaluation.value();
        return true;
    };

    float bestPitch = initialPitchGuess;
    float bestAbsoluteError = std::numeric_limits<float>::max();
    DragIntersection bestIntersection = {};
    bool bestIntersectionValid = false;

    DragIntersection initialIntersection = {};
    const bool initialIntersectionValid = evaluatePitch(initialPitchGuess, &initialIntersection);
    if (initialIntersectionValid)
    {
        bestIntersection = initialIntersection;
        bestPitch = initialPitchGuess;
        bestAbsoluteError = fabsf(initialIntersection.verticalError);
        bestIntersectionValid = true;
    }

    const float secantPitchDelta =
        initialPitchGuess >= 0.0f ? PITCH_SOLVER_DELTA_RADIANS : -PITCH_SOLVER_DELTA_RADIANS;
    const float secondPitch = initialPitchGuess + secantPitchDelta;
    DragIntersection secondIntersection = {};
    const bool secondIntersectionValid = evaluatePitch(secondPitch, &secondIntersection);
    if (secondIntersectionValid)
    {
        const float secondAbsoluteError = fabsf(secondIntersection.verticalError);
        if (!bestIntersectionValid || secondAbsoluteError < bestAbsoluteError)
        {
            bestPitch = secondPitch;
            bestIntersection = secondIntersection;
            bestAbsoluteError = secondAbsoluteError;
            bestIntersectionValid = true;
        }
    }

    if (initialIntersectionValid && secondIntersectionValid)
    {
        float previousPitch = initialPitchGuess;
        DragIntersection previousIntersection = initialIntersection;
        float currentPitch = secondPitch;
        DragIntersection currentIntersection = secondIntersection;

        for (uint8_t i = 0; i < solverConfig.pitchSolverIterations; i++)
        {
            const float errorDelta =
                currentIntersection.verticalError - previousIntersection.verticalError;
            if (fabsf(errorDelta) <= solverConfig.heightToleranceMeters)
            {
                break;
            }

            const float nextPitch =
                currentPitch -
                currentIntersection.verticalError * (currentPitch - previousPitch) / errorDelta;

            if (nextPitch <= minPitchSearch || nextPitch >= maxPitchSearch)
            {
                break;
            }

            DragIntersection nextIntersection = {};
            if (!evaluatePitch(nextPitch, &nextIntersection))
            {
                break;
            }

            const float nextAbsoluteError = fabsf(nextIntersection.verticalError);
            if (nextAbsoluteError < bestAbsoluteError)
            {
                bestPitch = nextPitch;
                bestIntersection = nextIntersection;
                bestAbsoluteError = nextAbsoluteError;
                bestIntersectionValid = true;
            }

            if (nextAbsoluteError <= solverConfig.heightToleranceMeters ||
                fabsf(nextPitch - currentPitch) <= solverConfig.angleToleranceRadians)
            {
                *pitchAngle = bestPitch;
                *timeOfFlight = bestIntersection.timeOfFlight;
                return true;
            }

            previousPitch = currentPitch;
            previousIntersection = currentIntersection;
            currentPitch = nextPitch;
            currentIntersection = nextIntersection;
        }
    }

    if (!bestIntersectionValid || bestAbsoluteError > solverConfig.heightToleranceMeters)
    {
        return false;
    }

    *pitchAngle = bestPitch;
    *timeOfFlight = bestIntersection.timeOfFlight;
    return true;
}

inline bool applySphereDragBallisticsCompensation(
    const tap::algorithms::ballistics::SecondOrderKinematicState &targetState,
    float launchSpeed,
    float *pitchAngle,
    float *yawAngle,
    float *timeOfFlight,
    float *distance,
    float pitchAxisOffset = 0.0f,
    const SphereProjectileModel &projectileModel = ROBO_MASTER_17MM_SPHERE,
    const SphereDragSolverConfig &solverConfig = DEFAULT_SPHERE_DRAG_SOLVER_CONFIG)
{
    if (pitchAngle == nullptr || yawAngle == nullptr || timeOfFlight == nullptr ||
        distance == nullptr)
    {
        return false;
    }

    float compensatedPitch = *pitchAngle;
    float compensatedTimeOfFlight = *timeOfFlight;
    modm::Vector3f compensatedTargetPosition = targetState.position;

    for (uint8_t i = 0; i < solverConfig.movingTargetIterations; i++)
    {
        compensatedTargetPosition = targetState.projectForward(compensatedTimeOfFlight);

        const float horizontalDistance = sqrtf(
            compensatedTargetPosition.x * compensatedTargetPosition.x +
            compensatedTargetPosition.y * compensatedTargetPosition.y);
        if (horizontalDistance <= 0.0f)
        {
            return false;
        }

        float solvedPitch = compensatedPitch;
        float solvedTimeOfFlight = compensatedTimeOfFlight;
        if (!solveSphereDragPitch(
                horizontalDistance,
                compensatedTargetPosition.z,
                launchSpeed,
                compensatedPitch,
                &solvedPitch,
                &solvedTimeOfFlight,
                pitchAxisOffset,
                projectileModel,
                solverConfig))
        {
            return false;
        }

        compensatedPitch = solvedPitch;
        compensatedTimeOfFlight = solvedTimeOfFlight;
    }

    *pitchAngle = compensatedPitch;
    *timeOfFlight = compensatedTimeOfFlight;
    *distance = compensatedTargetPosition.getLength();
    return true;
}
}  // namespace aruwsrc::algorithms

#endif  // SPHERICAL_PROJECTILE_AIM_HPP_
