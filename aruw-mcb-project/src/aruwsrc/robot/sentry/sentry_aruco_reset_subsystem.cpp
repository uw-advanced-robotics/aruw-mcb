#include "sentry_aruco_reset_subsystem.hpp"

#include "tap/algorithms/math_user_utils.hpp"

// #include "aruwsrc/control/turret/algorithms/world_frame_turret_yaw_controller.hpp"
#include "aruwsrc/control/turret/constants/turret_constants.hpp"
#include "modm/math/geometry/quaternion.hpp"
#include "modm/math/geometry/vector2.hpp"
#include "modm/math/geometry/vector3.hpp"

using namespace tap::algorithms::transforms;
using namespace aruwsrc::sentry;

SentryArucoResetSubsystem::SentryArucoResetSubsystem(
    tap::Drivers& drivers,
    aruwsrc::serial::VisionCoprocessor& vision,
    aruwsrc::sentry::SentryChassisWorldYawObserver& yawObserver,
    aruwsrc::sentry::SentryKFOdometry2DSubsystem& odometrySubsystem,
    SentryTransforms& transforms,
    aruwsrc::control::turret::algorithms::TurretMajorWorldFrameController& majorController)
    : tap::control::Subsystem(&drivers),
      vision(vision),
      yawObserver(yawObserver),
      odometrySubsystem(odometrySubsystem),
      transforms(transforms),
      majorController(majorController)
{
}

void SentryArucoResetSubsystem::refresh()
{
    const aruwsrc::serial::VisionCoprocessor::ArucoResetData& resetData =
        vision.getLastArucoResetData();

    if (!resetData.updated) return;
    vision.invalidateArucoResetData();

    modm::Quaternion<float> q(
        resetData.data.quatW,
        resetData.data.quatX,
        resetData.data.quatY,
        resetData.data.quatZ);

    const Transform& majorToMinor = transforms.getWorldToTurretMajor().getInverse().compose(
        transforms.getWorldToTurret(resetData.data.turretId));

    const Transform& chassisToMajor = transforms.getWorldToChassis().getInverse().compose(
        transforms.getWorldToTurretMajor().getInverse());

    float newYaw = tap::algorithms::eulerAnglesFromQuaternion(q).z - majorToMinor.getYaw() -
                   chassisToMajor.getYaw();

    float oldYaw;
    yawObserver.getChassisWorldYaw(&oldYaw);

    float chassisX = resetData.data.x -
                     transforms.getWorldToTurret(resetData.data.turretId).getX() +
                     transforms.getWorldToChassis().getX();
    float chassisY = resetData.data.y -
                     transforms.getWorldToTurret(resetData.data.turretId).getY() +
                     transforms.getWorldToChassis().getY();

    setOrientation(newYaw, oldYaw);
    setPosition(chassisX, chassisY)
}

void SentryArucoResetSubsystem::setOrientation(float newYaw, float oldYaw)
{
    odometrySubsystem.overrideOdometryOrientation(newYaw - oldYaw);
    yawObserver.overrideChassisYaw(newYaw);
}

void SentryArucoResetSubsystem::setPosition(const float x, const float y)
{
    odometrySubsystem.overrideOdometryPosition(modm::Vector2f(x, y));
}
