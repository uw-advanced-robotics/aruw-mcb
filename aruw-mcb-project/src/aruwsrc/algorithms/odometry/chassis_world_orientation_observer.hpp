/*
 * Copyright (c) 2020-2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef CHASSIS_WORLD_ORIENTATION_OBSERVER_HPP_
#define CHASSIS_WORLD_ORIENTATION_OBSERVER_HPP_

#include "tap/communication/sensors/imu/imu_interface.hpp"

#include "aruwsrc/algorithms/state/orientation_observer_interface.hpp"
#include "aruwsrc/communication/can/turret_mcb_can_comm.hpp"

namespace aruwsrc::algorithms::odometry
{

template <aruwsrc::algorithms::state::Frame ADJACENT>
class ChassisWorldOrientationObserver
    : public aruwsrc::algorithms::state::OrientationObserverInterface<
          aruwsrc::algorithms::state::Frame::WORLD,
          aruwsrc::algorithms::state::Frame::CHASSIS>
{
public:
    ChassisWorldOrientationObserver(
        const tap::communication::sensors::imu::ImuInterface& chassisImu,
        const aruwsrc::algorithms::state::OrientationObserverInterface<
            aruwsrc::algorithms::state::Frame::WORLD,
            ADJACENT>& adjacentImu,
        const aruwsrc::algorithms::state::OrientationObserverInterface<
            aruwsrc::algorithms::state::Frame::CHASSIS,
            ADJACENT>& adjacentEncoder)
        : chassisImu(chassisImu),
          adjacentImu(adjacentImu),
          adjacentEncoder(adjacentEncoder)
    {
    }

    tap::algorithms::transforms::DynamicOrientation getOrientation() const;

    bool isOnline() const override;

private:
    const tap::communication::sensors::imu::ImuInterface& chassisImu;
    const aruwsrc::algorithms::state::OrientationObserverInterface<
        aruwsrc::algorithms::state::Frame::WORLD,
        ADJACENT>& adjacentImu;
    const aruwsrc::algorithms::state::OrientationObserverInterface<
        aruwsrc::algorithms::state::Frame::CHASSIS,
        ADJACENT>& adjacentEncoder;
};  // ChassisWorldOrientationObserver

template <aruwsrc::algorithms::state::Frame ADJACENT>
tap::algorithms::transforms::DynamicOrientation ChassisWorldOrientationObserver<
    ADJACENT>::getOrientation() const
{
    if (adjacentImu.isOnline())
    {
        return adjacentImu.getOrientation().compose(adjacentEncoder.getOrientation().inverse());
    }
    return tap::algorithms::transforms::DynamicOrientation(
        chassisImu.getRoll(),
        chassisImu.getPitch(),
        chassisImu.getYaw(),
        chassisImu.getGx(),
        chassisImu.getGy(),
        chassisImu.getGz());
}

template <aruwsrc::algorithms::state::Frame ADJACENT>
bool ChassisWorldOrientationObserver<ADJACENT>::isOnline() const
{
    // This observer handles fallback behavior, so its promise is that it will always have a valid
    // estimate.
    return true;
}

}  // namespace aruwsrc::algorithms::odometry

#endif  // CHASSIS_WORLD_ORIENTATION_OBSERVER_HPP_