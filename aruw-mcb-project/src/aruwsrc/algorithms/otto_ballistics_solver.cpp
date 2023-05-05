#include "otto_ballistics_solver.hpp"


using namespace tap::algorithms;
namespace aruwsrc::algorithms
{

OttoBallisticsSolver::OttoBallisticsSolver(
    const tap::algorithms::odometry::Odometry2DInterface &odometryInterface,
    const aruwsrc::control::turret::SentryTurretMajorSubsystem &turretMajor,
    const aruwsrc::sentry::SentryTransforms &transforms,
    const control::launcher::LaunchSpeedPredictorInterface &frictionWheels,
    const float defaultLaunchSpeed,
    const float turretToMajorRadius,
    uint8_t turretID)
    : odometryInterface(odometryInterface),
      turretMajor(turretMajor),
      frictionWheels(frictionWheels),
      defaultLaunchSpeed(defaultLaunchSpeed),
      transforms(transforms),
      turretToMajorRadius(turretToMajorRadius),
      turretID(turretID)
{
}

std::optional<OttoBallisticsSolver::BallisticsSolution> OttoBallisticsSolver::
    computeTurretAimAngles(const aruwsrc::serial::VisionCoprocessor::TurretAimData& aimData)
{
    // Verify that CV is actually online and that the aimData had a target
    if (!aimData.pva.updated)
    {
        lastComputedSolution = std::nullopt;
        return std::nullopt;
    }

    if (lastAimDataTimestamp != aimData.timestamp ||
        lastOdometryTimestamp != transforms.lastComputedTimestamp())
    {
        lastAimDataTimestamp = aimData.timestamp;
        lastOdometryTimestamp = transforms.lastComputedTimestamp();

        // if the friction wheel launch speed is 0, use a default launch speed so ballistics
        // gives a reasonable computation
        float launchSpeed = frictionWheels.getPredictedLaunchSpeed();
        if (tap::algorithms::compareFloatClose(launchSpeed, 0.0f, 1e-5f))
        {
            launchSpeed = defaultLaunchSpeed;
        }

        modm::Vector3f turretPosition = modm::Vector3f(transforms.getWorldToTurretX(turretID), transforms.getWorldToTurretY(turretID), 0);
        const modm::Vector2f chassisVel = odometryInterface.getCurrentVelocity2D();

        // target state, frame whose axis is at the turret center and z is up
        // assume acceleration of the chassis is 0 since we don't measure it
        auto& worldToMajor = transforms.getWorldToTurretMajor();

        ballistics::MeasuredKinematicState targetState = {
            .position =
                {aimData.pva.xPos - turretPosition.x,
                 aimData.pva.yPos - turretPosition.y,
                 aimData.pva.zPos - turretPosition.z},
            .velocity =
                // chassis-forward is +x
                // someone needs to check my math on the below two calculations
                // @todo incorporate velocity into the transforms
                {aimData.pva.xVel - (chassisVel.x - turretMajor.yawMotor.getChassisFrameVelocity() * std::cos(worldToMajor.getYaw()) * turretToMajorRadius), // need to subtract out rotational velocity of major
                 aimData.pva.yVel - (chassisVel.y - turretMajor.yawMotor.getChassisFrameVelocity() * std::sin(worldToMajor.getYaw()) * turretToMajorRadius),
                 aimData.pva.zVel},
            .acceleration =
                {aimData.pva.xAcc,
                 aimData.pva.yAcc,
                 aimData.pva.zAcc},  // TODO consider using chassis
                                     // acceleration from IMU
        };

        // time in microseconds to project the target position ahead by
        int64_t projectForwardTimeDt =
            static_cast<int64_t>(tap::arch::clock::getTimeMicroseconds()) -
            static_cast<int64_t>(aimData.timestamp);

        // project the target position forward in time s.t. we are computing a ballistics solution
        // for a target "now" rather than whenever the camera saw the target
        targetState.position = targetState.projectForward(projectForwardTimeDt / 1E6f);

        lastComputedSolution = BallisticsSolution();
        lastComputedSolution->distance = targetState.position.getLength();

        if (!ballistics::findTargetProjectileIntersection(
                targetState,
                launchSpeed,
                3,
                &lastComputedSolution->pitchAngle,
                &lastComputedSolution->yawAngle,
                &lastComputedSolution->timeOfFlight,
                transforms.getWorldToTurretPitch(turretID)))
        {
            lastComputedSolution = std::nullopt;
        }
    }

    return lastComputedSolution;
}

}