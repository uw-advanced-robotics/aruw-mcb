#include "tap/architecture/clock.hpp"

#include "sentry_transforms.hpp"


using namespace tap::algorithms::transforms;

namespace aruwsrc::sentry
{

SentryTransforms::SentryTransforms(
    const tap::algorithms::odometry::Odometry2DInterface& chassisOdometry,
    const SentryTurretMajorSubsystem& turretMajor,
    const SentryTurretMinorSubsystem& turretMinorGirlboss,
    const SentryTurretMinorSubsystem& turretMinorMalewife,
    const SentryTransforms::TransformConfig& config)
    : chassisOdometry(chassisOdometry),
      turretMajor(turretMajor),
      turretMinorGirlboss(turretMinorGirlboss),
      turretMinorMalewife(turretMinorMalewife),
      config(config),
      turretMajorToTurretGirlboss(0., config.turretMinorOffset, 0., 0., 0., 0.),
      turretMajorToTurretMalewife(0., -config.turretMinorOffset, 0., 0., 0., 0.)
{
}

void SentryTransforms::updateTransforms()
{
    // World to Chassis
    // @todo we should discontinue using Location2D
    modm::Location2D chassisPose = chassisOdometry.getCurrentLocation2D();
    worldToChassis.updateTranslation(chassisPose.getX(), chassisPose.getY(), 0.);
    worldToChassis.updateRotation(0., 0., chassisPose.getOrientation());

    // Chassis to Turret Major
    chassisToTurretMajor.updateRotation(0., 0., turretMajor.yawMotor.getAngleFromCenter());

    // Turret Major to Minors
    // @todo have to access the damn motor inside because there's no getMotorEncoder or getMotorYaw in the RobotTurretSubsystem interface
    turretMajorToTurretGirlboss.updateRotation(0., turretMinorGirlboss.pitchMotor.getAngleFromCenter(), turretMinorGirlboss.yawMotor.getAngleFromCenter());
    turretMajorToTurretMalewife.updateRotation(0., turretMinorMalewife.pitchMotor.getAngleFromCenter(), turretMinorMalewife.yawMotor.getAngleFromCenter());

    // World transforms
    worldToTurretMajor = compose(worldToChassis, chassisToTurretMajor);
    worldToTurretGirlboss = compose(worldToTurretMajor, turretMajorToTurretGirlboss);
    worldToTurretMalewife = compose(worldToTurretMajor, turretMajorToTurretMalewife);

    lastComputedTime = tap::arch::clock::getTimeMicroseconds();  // @todo not necessarily the best way

    // Alternatively, we use the imus
}

SentryTransformsSubsystem::SentryTransformsSubsystem(
    tap::Drivers& drivers,
    const tap::algorithms::odometry::Odometry2DInterface& chassisOdometry,
    const SentryTurretMajorSubsystem& turretMajor,
    const SentryTurretMinorSubsystem& turretMinorGirlboss,
    const SentryTurretMinorSubsystem& turretMinorMalewife,
    const SentryTransforms::TransformConfig& config)
    : tap::control::Subsystem(&drivers),
      SentryTransforms(
        chassisOdometry,
        turretMajor,
        turretMinorGirlboss,
        turretMinorMalewife,
        config)
{
}

void SentryTransformsSubsystem::refresh()
{
    updateTransforms();
}

}  // namespace aruwsrc::sentry