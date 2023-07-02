#include "sentry_aruco_reset_subsystem.hpp"

#include "aruwsrc/control/turret/constants/turret_constants.hpp"
#include "modm/math/geometry/vector2.hpp"
#include "modm/math/geometry/vector3.hpp"
#include "aruwsrc/control/turret/algorithms/world_frame_turret_yaw_controller.hpp"

using namespace tap::algorithms::transforms;

namespace aruwsrc::sentry
{
SentryArucoResetSubsystem::SentryArucoResetSubsystem(
    tap::Drivers& drivers,
    SentryChassisWorldYawObserver& yawObserver,
    SentryKFOdometry2DSubsystem& odom,
    aruwsrc::serial::VisionCoprocessor& vcpp,
    SentryTransforms& transforms,
    aruwsrc::control::turret::algorithms::WorldFrameTurretYawCascadePIDController& majorController)
    : tap::control::Subsystem(&drivers),
      yawObserver(yawObserver),
      odom(odom),
      vcpp(vcpp),
      transforms(transforms),
      majorController(majorController)
{
}

void SentryArucoResetSubsystem::refresh()
{
    // pull from vision coprocessor, if updated we reset certain things
    const aruwsrc::serial::VisionCoprocessor::ArucoResetData& resetData = vcpp.getLastArucoResetData();
    
    if (resetData.updated)
    {
        // @todo bypassing incomplete transform system
        modm::Vector3f newPose(resetData.x, resetData.y, resetData.z);
        float newYaw = getEulerAngles(resetData).z - transforms.getMajorToMinor(resetData.turretId).getYaw() - transforms.getChassisToTurretMajor().getYaw();
        arucoYaw = newYaw;
        debug1 = transforms.getMajorToMinor(resetData.turretId).getYaw();
        debug2 = transforms.getChassisToTurretMajor().getYaw();
        

        // float oldYaw;
        yawObserver.getChassisWorldYaw(&oldYaw);
        // transformWorldOdomToChassis(newYaw, newPose, resetData.turretId);

        float chassisX = resetData.x - transforms.getWorldToMinor(resetData.turretId).getX() + transforms.getWorldToChassis().getX();
        float chassisY = resetData.y - transforms.getWorldToMinor(resetData.turretId).getY() + transforms.getWorldToChassis().getY();

        resetPosition(chassisX, chassisY);
        // resetOrientation(newYaw, oldYaw);
    }
}

void SentryArucoResetSubsystem::resetOrientation(float newYaw, float oldYaw)
{
    odom.overrideOdometryOrientation(newYaw - oldYaw);
    yawObserver.overrideChassisYaw(newYaw - oldYaw);

    float curMajorSetPoint = majorController.getSetpoint();

    // majorController.setSetpoint(curMajorSetPoint - (newYaw - oldYaw));
}

void SentryArucoResetSubsystem::SentryArucoResetSubsystem::resetPosition(
    const float x, const float y)
{
    // need to rely on this getting called before
    modm::Vector2f newPos(x, y);
    odom.overrideOdometryPosition(newPos);
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