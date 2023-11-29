/*
 * Copyright (c) 2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef SENTRY_REQUEST_SUBSYSTEM_HPP_
#define SENTRY_REQUEST_SUBSYSTEM_HPP_

#include "tap/control/subsystem.hpp"
#include "tap/util_macros.hpp"

#include "aruwsrc/algorithms/odometry/otto_chassis_world_yaw_observer.hpp"
#include "aruwsrc/robot/sentry/sentry_otto_kf_odometry_2d_subsystem.hpp"
#include "modm/math/geometry/quaternion.hpp"
#include "modm/math/geometry/vector3.hpp"

#include "vision_coprocessor.hpp"

namespace aruwsrc
{
class Drivers;
}

namespace aruwsrc::communication::serial
{
class SentryArucoResetSubsystem : public tap::control::Subsystem
{
public:
    SentryArucoResetSubsystem(
        tap::Drivers *drivers,
        aruwsrc::serial::VisionCoprocessor &visionCoprocessor,
        // Probably wrong
        aruwsrc::algorithms::odometry::OttoChassisWorldYawObserver &yawObserver,
        aruwsrc::algorithms::odometry::SentryOttoKFOdometry2DSubsystem &odometry2DSubsystem);

    void refresh() override;
    void initialize() override{};
    const char* getName() override { return "Sentry ARUCO Reset Subsystem"; }



private:
    aruwsrc::serial::VisionCoprocessor &sentryVisionCoprocessor;
    aruwsrc::algorithms::odometry::OttoChassisWorldYawObserver &yawObserver;
    aruwsrc::algorithms::odometry::SentryOttoKFOdometry2DSubsystem &odometry2DSubsystem;

    // returns vector3f of {x:roll, y:pitch: z: yaw}
    // @todo make this part of tap
    static modm::Vector3f getEulerAngles(
        const aruwsrc::serial::VisionCoprocessor::ArucoResetData &resetData)
    {
        modm::Vector3f eulerAngles;
        modm::Quaternion q(
            resetData.message.quat_w,
            resetData.message.quat_x,
            resetData.message.quat_y,
            resetData.message.quat_z);

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
};
}  // namespace aruwsrc::communication::serial

#endif  // SENTRY_REQUEST_SUBSYSTEM_HPP_
