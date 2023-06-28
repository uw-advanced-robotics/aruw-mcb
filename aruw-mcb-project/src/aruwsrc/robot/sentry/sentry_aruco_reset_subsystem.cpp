#include "sentry_aruco_reset_subsystem.hpp"

#include "aruwsrc/control/turret/constants/turret_constants.hpp"
#include "modm/math/geometry/vector2.hpp"
#include "modm/math/geometry/vector3.hpp"

using namespace tap::algorithms::transforms;

namespace aruwsrc::sentry
{
SentryArucoResetSubsystem::SentryArucoResetSubsystem(
    tap::Drivers& drivers,
    SentryChassisWorldYawObserver& yawObserver,
    SentryKFOdometry2DSubsystem& odom,
    aruwsrc::serial::VisionCoprocessor& vcpp,
    SentryTransforms& transforms)
    : tap::control::Subsystem(&drivers),
      yawObserver(yawObserver),
      odom(odom),
      vcpp(vcpp),
      transforms(transforms)
{
}

void SentryArucoResetSubsystem::refresh()
{
    // pull from vision coprocessor, if updated we reset certain things
    const VisionCoprocessor::ArucoResetData& resetData = vcpp.getLastArucoResetData();
    if (resetData.updated)
    {
        modm::Vector3f newPose(resetData.x, resetData.y, resetData.z);
        float newYaw = getEulerAngles(resetData).z;

        float oldYaw;
        yawObserver.getChassisWorldYaw(&oldYaw);
        transformWorldOdomToChassis(newYaw, newPose, resetData.turretId);

        resetPosition(resetData, oldYaw, oldYaw);
        // resetPosition(resetData, newYaw, oldYaw);
        // resetOrientation(resetData, newYaw);
    }
}

void SentryArucoResetSubsystem::resetOrientation(const VisionCoprocessor::ArucoResetData& resetData, float newYaw)
{
    yawObserver.overrideChassisYaw(newYaw);
}

void SentryArucoResetSubsystem::SentryArucoResetSubsystem::resetPosition(
    const VisionCoprocessor::ArucoResetData& resetData,
    float newYaw,
    float oldYaw)
{
    // need to rely on this getting called before
    modm::Vector2f newPos(resetData.x, resetData.y);
    odom.overrideOdometry(newPos, newYaw - oldYaw);
}

void SentryArucoResetSubsystem::transformWorldOdomToChassis(
    float& yaw,
    modm::Vector3f& pose,
    uint8_t turretID)
{
    // @todo: get inverse broken!!!!!!!!!!!!!!!!!!!!!!
    // auto& worldToMinor = transforms.getWorldToMinor(turretID);

    auto& majorToMinor = transforms.getMajorToMinor(turretID);
    auto& chassisToMajor = transforms.getChassisToTurretMajor();

    // @odo: notices all these plus signs!
    // yaw -= worldToMinor.getYaw();
    // minor to major
    yaw += majorToMinor.getYaw();
    // major to chassis
    yaw += chassisToMajor.getYaw();

    // pose.x -= worldToMinor.getX() + majorToMinor.getX() + chassisToMajor.getX();
    // pose.y -= worldToMinor.getY() + majorToMinor.getY() + chassisToMajor.getY();
    pose.x -= majorToMinor.getX() + chassisToMajor.getX();
    pose.y -= majorToMinor.getY() + chassisToMajor.getY();
}

}  // namespace aruwsrc::sentry