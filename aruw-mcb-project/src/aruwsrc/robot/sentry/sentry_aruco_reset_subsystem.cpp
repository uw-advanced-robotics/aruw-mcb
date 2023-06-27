#include "sentry_aruco_reset_subsystem.hpp"
#include "modm/math/geometry/vector3.hpp"
#include "modm/math/geometry/vector2.hpp"



namespace aruwsrc::sentry {
SentryArucoResetSubsystem::SentryArucoResetSubsystem(
        tap::Drivers& drivers,
        SentryChassisWorldYawObserver& yawObserver,
        SentryKFOdometry2DSubsystem& odom,
    aruwsrc::serial::VisionCoprocessor& vcpp

) :
tap::control::Subsystem(&drivers),
 yawObserver(yawObserver), odom(odom), vcpp(vcpp) {}

void SentryArucoResetSubsystem::refresh() {
    // pull from vision coprocessor, if updated we reset certain things
    const VisionCoprocessor::ArucoResetData& resetData = vcpp.getLastArucoResetData();
    if (resetData.updated) {
        float oldYaw;
        yawObserver.getChassisWorldYaw(&oldYaw);
        resetPosition(resetData, oldYaw);
        resetOrientation(resetData);
    }
}


void SentryArucoResetSubsystem::resetOrientation(const VisionCoprocessor::ArucoResetData& resetData) {
    modm::Vector3f eulerAngles = getEulerAngles(resetData);
    float& yaw = eulerAngles.z; 
    yawObserver.overrideChassisYaw(yaw);
}

void SentryArucoResetSubsystem::resetPosition(const VisionCoprocessor::ArucoResetData& resetData, float oldYaw) {
    // need to rely on this getting called before 
    modm::Vector3f eulerAngles = getEulerAngles(resetData);
    float& newYaw = eulerAngles.z; 
    modm::Vector2f newPos(resetData.x, resetData.y);

    odom.overrideOdometry(newPos, newYaw - oldYaw);
}



}