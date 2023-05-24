
#ifndef SENTRY_TRANSFORMS_HPP_
#define SENTRY_TRANSFORMS_HPP_

#include "tap/algorithms/transforms/transform.hpp"
#include "tap/algorithms/transforms/frame.hpp"
#include "tap/algorithms/odometry/odometry_2d_interface.hpp"
#include "tap/motor/motor_interface.hpp"

#include "tap/control/subsystem.hpp"

#include "aruwsrc/robot/sentry/sentry_turret_major_subsystem.hpp"
#include "aruwsrc/robot/sentry/sentry_turret_minor_subsystem.hpp"

using namespace tap::algorithms::transforms;

using namespace aruwsrc::control::turret;
namespace aruwsrc::sentry
{

// Design decision: all frames are dependent of the robot design and thus should be defined in robot
class WorldFrame : Frame {};
class ChassisFrame : Frame {};
class TurretMajorFrame : Frame {};
class TurretMinorFrame : Frame {};
class TurretMinorGirlbossFrame : public TurretMinorFrame {};
class TurretMinorMalewifeFrame : public TurretMinorFrame {};
class CameraFrame : Frame {};


// @todo incorporate velocities? for example, the otto ballistics solver requires chassis velocity
class SentryTransforms
{
public:
    // @todo i'm not sure if we'll need to compute accurate z-values or not; probably not but we should definitely in the future
    struct TransformConfig
    {
        // Offset from turret minor yaw axis to turret major yaw axis (should only be in the y-direction of the turret major frame)
        const float minorToMajorRadius;
        const float minorToCameraXOffset;
        const float minorToCameraYOffset;
        const float minorToCameraZOffset;
    };

    SentryTransforms(
        const tap::algorithms::odometry::Odometry2DInterface& chassisOdometry,
        const SentryTurretMajorSubsystem& turretMajor,
        const SentryTurretMinorSubsystem& turretMinorGirlboss,
        const SentryTurretMinorSubsystem& turretMinorMalewife,
        const TransformConfig& config);

    void updateTransforms();

    inline const Transform<WorldFrame, ChassisFrame>& getWorldToChassis() const { return worldToChassis; };
    inline const Transform<WorldFrame, TurretMajorFrame>& getWorldToTurretMajor() const { return worldToTurretMajor; };
    inline const Transform<WorldFrame, TurretMinorGirlbossFrame>& getWorldToTurretGirlboss() const { return worldToTurretGirlboss; };
    inline const Transform<WorldFrame, TurretMinorMalewifeFrame>& getWorldToTurretMalewife() const { return worldToTurretMalewife; };
    inline const Transform<TurretMinorGirlbossFrame, CameraFrame>& getTurretGirlbossToCamera() const { return turretGirlbossToCamera; };
    inline const Transform<TurretMinorMalewifeFrame, CameraFrame>& getTurretMalewifeToCamera() const { return turretMalewifeToCamera; };
    inline const Transform<TurretMajorFrame, TurretMinorGirlbossFrame>& getTurretMajorToTurretGirlboss() const { return turretMajorToTurretGirlboss; };
    inline const Transform<TurretMajorFrame, TurretMinorMalewifeFrame>& getTurretMajorToTurretMalewife() const { return turretMajorToTurretMalewife; };
    inline const Transform<ChassisFrame, TurretMajorFrame>& getChassisToTurretMajor() const { return chassisToTurretMajor; };

    inline uint32_t lastComputedTimestamp() const { return lastComputedTime; };

private:
    TransformConfig config;
    // @todo move to odometry class
    const tap::algorithms::odometry::Odometry2DInterface& chassisOdometry;
    const SentryTurretMajorSubsystem& turretMajor;
    const SentryTurretMinorSubsystem& turretMinorGirlboss;
    const SentryTurretMinorSubsystem& turretMinorMalewife;

    // Transforms
    Transform<WorldFrame, ChassisFrame> worldToChassis;
    Transform<WorldFrame, TurretMajorFrame> worldToTurretMajor;
    Transform<WorldFrame, TurretMinorGirlbossFrame> worldToTurretGirlboss;
    Transform<WorldFrame, TurretMinorMalewifeFrame> worldToTurretMalewife;

    // Intermediary transforms
    Transform<ChassisFrame, TurretMajorFrame> chassisToTurretMajor;
    Transform<TurretMajorFrame, TurretMinorGirlbossFrame> turretMajorToTurretGirlboss;
    Transform<TurretMajorFrame, TurretMinorMalewifeFrame> turretMajorToTurretMalewife;
    Transform<TurretMinorGirlbossFrame, CameraFrame> turretGirlbossToCamera;
    Transform<TurretMinorMalewifeFrame, CameraFrame> turretMalewifeToCamera;

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