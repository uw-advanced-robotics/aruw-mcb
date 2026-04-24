#ifndef SPHERICAL_PROJECTILE_AIM_HPP_
#define SPHERICAL_PROJECTILE_AIM_HPP_

#include <cmath>
#include <cstdint>
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

struct SphereDragCorrectionConfig
{
    float constantPitchOffsetRadians;
    float verticalErrorPitchGain;
    float maxAdditionalPitchCorrectionRadians;
};

static constexpr SphereDragCorrectionConfig DEFAULT_SPHERE_DRAG_CORRECTION_CONFIG = {
    .constantPitchOffsetRadians = 0.0f,
    .verticalErrorPitchGain = 0.0f,
    .maxAdditionalPitchCorrectionRadians = 0.0f,
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
    const SphereDragCorrectionConfig &correctionConfig = DEFAULT_SPHERE_DRAG_CORRECTION_CONFIG)
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

    static constexpr float TIMESTEP_SECONDS = 0.002f;
    static constexpr float MAX_SIMULATION_TIME_SECONDS = 2.0f;

    for (float timeSeconds = TIMESTEP_SECONDS;
         timeSeconds <= MAX_SIMULATION_TIME_SECONDS;
         timeSeconds += TIMESTEP_SECONDS)
    {
        const SphereProjectileState previousState = projectileState;
        projectileState = rungeKuttaIntegrateProjectileState(
            projectileState,
            TIMESTEP_SECONDS,
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
                .timeOfFlight = previousTimeSeconds + interpolationRatio * TIMESTEP_SECONDS,
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

inline bool applySphereDragBallisticsCompensation(
    const tap::algorithms::ballistics::SecondOrderKinematicState &targetState,
    float launchSpeed,
    float *pitchAngle,
    float *yawAngle,
    float *timeOfFlight,
    float *distance,
    float pitchAxisOffset = 0.0f,
    const SphereProjectileModel &projectileModel = ROBO_MASTER_17MM_SPHERE,
    const SphereDragCorrectionConfig &correctionConfig = DEFAULT_SPHERE_DRAG_CORRECTION_CONFIG)
{
    if (pitchAngle == nullptr || yawAngle == nullptr || timeOfFlight == nullptr ||
        distance == nullptr)
    {
        return false;
    }

    const modm::Vector3f compensatedTargetPosition = targetState.projectForward(*timeOfFlight);
    const float horizontalDistance = sqrtf(
        compensatedTargetPosition.x * compensatedTargetPosition.x +
        compensatedTargetPosition.y * compensatedTargetPosition.y);
    if (horizontalDistance <= 0.0f)
    {
        return false;
    }

    if (!isfinite(correctionConfig.constantPitchOffsetRadians))
    {
        return false;
    }

    (void)yawAngle;
    *distance = compensatedTargetPosition.getLength();

    float pitchCorrection = correctionConfig.constantPitchOffsetRadians;

    if (correctionConfig.verticalErrorPitchGain > 0.0f &&
        correctionConfig.maxAdditionalPitchCorrectionRadians > 0.0f)
    {
        const auto baseIntersection = simulateSphereDragIntersection(
            horizontalDistance,
            compensatedTargetPosition.z,
            launchSpeed,
            *pitchAngle,
            pitchAxisOffset,
            projectileModel,
            correctionConfig);
        if (baseIntersection.has_value())
        {
            float additionalPitchCorrection =
                -correctionConfig.verticalErrorPitchGain * baseIntersection->verticalError /
                horizontalDistance;
            additionalPitchCorrection = fmaxf(
                -correctionConfig.maxAdditionalPitchCorrectionRadians,
                fminf(
                    correctionConfig.maxAdditionalPitchCorrectionRadians,
                    additionalPitchCorrection));
            pitchCorrection += additionalPitchCorrection;
        }
    }

    *pitchAngle += pitchCorrection;
    return true;
}
}  // namespace aruwsrc::algorithms

#endif  // SPHERICAL_PROJECTILE_AIM_HPP_
