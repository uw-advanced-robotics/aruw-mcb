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

#ifndef OTTO_DISPLACEMENT_ODOMETRY_2D_SUBSYSTEM_HPP_
#define OTTO_DISPLACEMENT_ODOMETRY_2D_SUBSYSTEM_HPP_

#include "tap/algorithms/odometry/odometry_2d_interface.hpp"
#include "tap/algorithms/odometry/odometry_2d_tracker.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/control/turret_subsystem_interface.hpp"

#include "aruwsrc/algorithms/odometry/otto_chassis_world_yaw_observer.hpp"
#include "aruwsrc/control/chassis/chassis_subsystem.hpp"
#include "aruwsrc/drivers.hpp"

namespace aruwsrc::algorithms::odometry
{
/**
 * Subsystem for recording chassis 2D displacement using motor angular displacement.
 * Should be refreshed regularly for best results.
 */
class OttoDisplacementOdometry2DSubsystem : public tap::control::Subsystem,
                                            public tap::algorithms::odometry::Odometry2DInterface
{
public:
    /**
     * @param[in] drivers pointer to aruwsrc drivers
     * @param[in] turret pointer to object that implements TurretSubsystemInterface
     * @param[in] chassis pointer to aruwsrc ChassisSubsystem
     */
    OttoDisplacementOdometry2DSubsystem(
        aruwsrc::Drivers* drivers,
        tap::control::turret::TurretSubsystemInterface* const turret,
        aruwsrc::chassis::ChassisSubsystem* const chassis);

    void refresh() override;

    modm::Location2D<float> getCurrentLocation2D() const override
    {
        return odometryTracker.getCurrentLocation2D();
    }

private:
    tap::algorithms::odometry::Odometry2DTracker odometryTracker;
    OttoChassisWorldYawObserver orientationObserver;
    aruwsrc::chassis::ChassisSubsystem* const chassis;
};

}  // namespace aruwsrc::algorithms::odometry

#endif  // OTTO_DISPLACEMENT_ODOMETRY_2D_SUBSYSTEM_HPP_
