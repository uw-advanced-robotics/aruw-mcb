#ifndef SENTRY_ARUCO_RESET_SUBSYSTEM_HPP_
#define SENTRY_ARUCO_RESET_SUBSYSTEM_HPP_

#include "tap/control/subsystem.hpp"

#include "aruwsrc/communication/serial/vision_coprocessor.hpp"
#include "aruwsrc/robot/sentry/sentry_chassis_world_yaw_observer.hpp"
#include "aruwsrc/robot/sentry/sentry_kf_odometry_2d_subsystem.hpp"
#include "aruwsrc/robot/sentry/sentry_transforms.hpp"
#include "aruwsrc/robot/sentry/sentry_turret_major_world_relative_yaw_controller.hpp"
#include "modm/math/geometry/quaternion.hpp"
#include "modm/math/geometry/vector3.hpp"

namespace aruwsrc::sentry
{

class SentryArucoResetSubsystem : public tap::control::Subsystem
{
public:
    SentryArucoResetSubsystem(
        tap::Drivers& drivers,
        aruwsrc::serial::VisionCoprocessor& vision,
        aruwsrc::sentry::SentryChassisWorldYawObserver& yawObserver,
        aruwsrc::sentry::SentryKFOdometry2DSubsystem& odometrySubsystem,
        SentryTransforms& transforms,
        aruwsrc::control::turret::algorithms::TurretMajorWorldFrameController& majorController);

    void initialize() override {};

    void refresh() override;

    const char* getName() override { return "Sentry Aruco Reset Subsystem"; }

    mockable inline bool isOnline() const { return true; }

private:
    aruwsrc::serial::VisionCoprocessor& vision;
    SentryChassisWorldYawObserver& yawObserver;
    SentryKFOdometry2DSubsystem& odometrySubsystem;
    const SentryTransforms& transforms;
    aruwsrc::control::turret::algorithms::TurretMajorWorldFrameController& majorController;

    void setOrientation(float newYaw, float oldYaw);
    void setPosition(float x, float y);
};

}  // namespace aruwsrc::sentry

#endif  // SENTRY_ARUCO_RESET_SUBSYSTEM_HPP_