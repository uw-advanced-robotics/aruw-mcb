/*
 * Copyright (c) 2023-2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "aruwsrc/communication/serial/vision_coprocessor.hpp"
#include "aruwsrc/robot/sentry/sentry_chassis_world_yaw_observer.hpp"
#include "aruwsrc/robot/sentry/sentry_kf_odometry_2d_subsystem.hpp"
#include "aruwsrc/robot/sentry/sentry_transforms.hpp"

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
        SentryTransforms& transforms);

    void initialize() override{};

    void refresh() override;

    const char* getName() const { return "Sentry Aruco Reset Subsystem"; }

    mockable inline bool isOnline() const { return true; }

private:
    aruwsrc::serial::VisionCoprocessor& vision;
    SentryChassisWorldYawObserver& yawObserver;
    SentryKFOdometry2DSubsystem& odometrySubsystem;
    const SentryTransforms& transforms;

    void setOrientation(float newYaw, float oldYaw);
    void setPosition(float x, float y);
};

}  // namespace aruwsrc::sentry

#endif  // SENTRY_ARUCO_RESET_SUBSYSTEM_HPP_