
#ifndef SENTRY_TRANSFORMS_HPP_
#define SENTRY_TRANSFORMS_HPP_

#include "tap/algorithms/transforms/transform.hpp"
#include "tap/algorithms/transforms/frame.hpp"
#include "tap/algorithms/odometry/odometry_2d_interface.hpp"
#include "tap/motor/motor_interface.hpp"

#include "tap/control/subsystem.hpp"

#include "aruwsrc/robot/sentry/sentry_turret_major_subsystem.hpp"
#include "aruwsrc/robot/sentry/sentry_turret_minor_subsystem.hpp"

using namespace aruwsrc::control::turret;
namespace aruwsrc::sentry
{

// Design decision: all frames are dependent of the robot design and thus should be defined in robot
class WorldFrame : tap::algorithms::transforms::Frame {};
class ChassisFrame : tap::algorithms::transforms::Frame {};
class TurretMajorFrame : tap::algorithms::transforms::Frame {};
class TurretMinorGirlbossFrame : tap::algorithms::transforms::Frame {};
class TurretMinorMalewifeFrame : tap::algorithms::transforms::Frame {};


// @todo incorporate velocities? for example, the otto ballistics solver requires chassis velocity
// @todo remove templates; makes it impossible for other classes to internally store references to transforms
class SentryTransforms
{
public:
    // @todo i'm not sure if we'll need to compute accurate z-values or not; probably not but we should definitely in the future
    struct TransformConfig
    {
        // Offset from turret minor yaw axis to turret major yaw axis (should only be in the y-direction of the turret major frame)
        const float turretMinorOffset;
    };

    SentryTransforms(
        const tap::algorithms::odometry::Odometry2DInterface& chassisOdometry,
        const SentryTurretMajorSubsystem& turretMajor,
        const SentryTurretMinorSubsystem& turretMinorGirlboss,
        const SentryTurretMinorSubsystem& turretMinorMalewife,
        const TransformConfig& config);

    void updateTransforms();

    inline const tap::algorithms::transforms::Transform<WorldFrame, ChassisFrame>& getWorldToChassis() const { return worldToChassis; };
    inline const tap::algorithms::transforms::Transform<WorldFrame, TurretMajorFrame>& getWorldToTurretMajor() const { return worldToTurretMajor; };
    inline const tap::algorithms::transforms::Transform<WorldFrame, TurretMinorGirlbossFrame>& getWorldToTurretGirlboss() const { return worldToTurretGirlboss; };
    inline const tap::algorithms::transforms::Transform<WorldFrame, TurretMinorMalewifeFrame>& getWorldToTurretMalewife() const { return worldToTurretMalewife; };

    // @todo EXTREMELY HACKY SOLUTION for the fact that transforms cannot be passed into otto ballistics solver without screwing over everything
    inline const float& getWorldToTurretX(uint8_t turretID) const { return (turretID == 0) ? worldToTurretMalewife.getX() : worldToTurretGirlboss.getX(); };
    inline const float& getWorldToTurretY(uint8_t turretID) const { return (turretID == 0) ? worldToTurretMalewife.getY() : worldToTurretGirlboss.getY(); };
    inline const float& getWorldToTurretPitch(uint8_t turretID) const { return (turretID == 0) ? worldToTurretMalewife.getPitch() : worldToTurretGirlboss.getPitch(); };

    inline const uint32_t lastComputedTimestamp() const { return lastComputedTime; };

private:
    TransformConfig config;

    // Transforms
    tap::algorithms::transforms::Transform<WorldFrame, ChassisFrame> worldToChassis;
    tap::algorithms::transforms::Transform<WorldFrame, TurretMajorFrame> worldToTurretMajor;
    tap::algorithms::transforms::Transform<WorldFrame, TurretMinorGirlbossFrame> worldToTurretGirlboss;
    tap::algorithms::transforms::Transform<WorldFrame, TurretMinorMalewifeFrame> worldToTurretMalewife;

    // Intermediary transforms
    tap::algorithms::transforms::Transform<ChassisFrame, TurretMajorFrame> chassisToTurretMajor;
    tap::algorithms::transforms::Transform<TurretMajorFrame, TurretMinorGirlbossFrame> turretMajorToTurretGirlboss;
    tap::algorithms::transforms::Transform<TurretMajorFrame, TurretMinorMalewifeFrame> turretMajorToTurretMalewife;

    // @todo move to odometry class
    const tap::algorithms::odometry::Odometry2DInterface& chassisOdometry;
    const SentryTurretMajorSubsystem& turretMajor;
    const SentryTurretMinorSubsystem& turretMinorGirlboss;
    const SentryTurretMinorSubsystem& turretMinorMalewife;

    uint32_t lastComputedTime = 0;
};


class SentryTransformsSubsystem : public tap::control::Subsystem, public SentryTransforms
{
public:
    SentryTransformsSubsystem(
        tap::Drivers &drivers,
        const tap::algorithms::odometry::Odometry2DInterface& chassisOdometry,
        const SentryTurretMajorSubsystem& turretMajor,
        const SentryTurretMinorSubsystem& turretMinorGirlboss,
        const SentryTurretMinorSubsystem& turretMinorMalewife,
        const TransformConfig& config);
    
    void refresh() override;
};

}  // namespace aruwsrc::sentry

#endif  // SENTRY_TRANSFORMS_HPP_