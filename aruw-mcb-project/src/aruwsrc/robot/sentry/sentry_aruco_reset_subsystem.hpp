/*
 * Copyright (c) 2020-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruw-mcb.
 *
 * aruw-mcb is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aruw-mcb is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aruw-mcb.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef SENTRY_ARUCO_RESET_SUBSYSTEM_HPP_
#define SENTRY_ARUCO_RESET_SUBSYSTEM_HPP_

#include "tap/control/subsystem.hpp"
#include "aruwsrc/robot/sentry/sentry_kf_odometry_2d_subsystem.hpp"
#include "aruwsrc/robot/sentry/sentry_chassis_world_yaw_observer.hpp"
#include "aruwsrc/communication/serial/vision_coprocessor.hpp"
#include "modm/math/geometry/vector3.hpp"
#include "modm/math/geometry/quaternion.hpp"
#include "aruwsrc/robot/sentry/sentry_transforms.hpp"
#include "aruwsrc/control/turret/algorithms/world_frame_turret_yaw_controller.hpp"


namespace aruwsrc::sentry {

class SentryArucoResetSubsystem : public tap::control::Subsystem
{
public:
    /**
     * WHO CARES!!!!!
     */
    SentryArucoResetSubsystem(
        tap::Drivers& drivers,
        SentryChassisWorldYawObserver& yawObserver,
        SentryKFOdometry2DSubsystem& odom,
        aruwsrc::serial::VisionCoprocessor& vcpp,
        SentryTransforms& transforms,
        aruwsrc::control::turret::algorithms::WorldFrameTurretYawCascadePIDController& majorController);

    void initialize() override {};

    void refresh() override;

    const char* getName() override { return "Turret"; }

    mockable inline bool isOnline() const {return true;} // haha

private:
    SentryChassisWorldYawObserver& yawObserver;
    SentryKFOdometry2DSubsystem& odom;
    const aruwsrc::serial::VisionCoprocessor& vcpp;
    const SentryTransforms& transforms;
    aruwsrc::control::turret::algorithms::WorldFrameTurretYawCascadePIDController& majorController;

    void resetOrientation(float newYaw, float oldYaw);

    void resetPosition(const float x, const float y);

    float arucoYaw;
    float oldYaw;
    float debug1;
    float debug2;


    void transformWorldOdomToChassis(
        float& worldYaw,
        modm::Vector3f& worldPose,
        uint8_t turretID
    );

    modm::Vector3f turretPosToChassisPos(const aruwsrc::serial::VisionCoprocessor::ArucoResetData& resetData);

    // returns vector3f of {x:roll, y:pitch: z: yaw}
    // @todo make this part of tap
    static modm::Vector3f getEulerAngles(const aruwsrc::serial::VisionCoprocessor::ArucoResetData& resetData)  {
        modm::Vector3f eulerAngles;
        modm::Quaternion q(resetData.quatW, resetData.quatX, resetData.quatY, resetData.quatZ);

        // from wikipedia :P
        // roll (x-axis rotation)
        double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
        double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
        eulerAngles.x = std::atan2(sinr_cosp, cosr_cosp);

        // pitch (y-axis rotation)
        double sinp = std::sqrt(1 + 2 * (q.w * q.y - q.x * q.z));
        double cosp = std::sqrt(1 - 2 * (q.w * q.y - q.x * q.z));
        eulerAngles.y = 2 * std::atan2(sinp, cosp) - M_PI / 2;

        // yaw (z-axis rotation)
        double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
        eulerAngles.z = std::atan2(siny_cosp, cosy_cosp);

        return eulerAngles;
    }


};  // class SentryArucoResetSubsystem

}  // namespace aruwsrc::sentry

#endif  // SENTRY_ARUCO_RESET_SUBSYSTEM_HPP_
