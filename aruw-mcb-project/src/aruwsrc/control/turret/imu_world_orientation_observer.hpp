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

#ifndef IMU_WORLD_ORIENTATION_OBSERVER_HPP_
#define IMU_WORLD_ORIENTATION_OBSERVER_HPP_

#include "tap/communication/sensors/imu/imu_interface.hpp"

#include "aruwsrc/algorithms/state/orientation_observer_interface.hpp"

namespace aruwsrc::control::turret
{
template <aruwsrc::algorithms::state::Frame MOUNTING_FRAME>
class ImuWorldOrientationObserver
    : public aruwsrc::algorithms::state::
          OrientationObserverInterface<aruwsrc::algorithms::state::Frame::WORLD, MOUNTING_FRAME>
{
public:
    ImuWorldOrientationObserver(const tap::communication::sensors::imu::ImuInterface& imu);

    tap::algorithms::transforms::DynamicOrientation getOrientation() const override;

    virtual bool isOnline() const override;

private:
    const tap::communication::sensors::imu::ImuInterface& imu;
};  // ImuWorldOrientationObserver

template <aruwsrc::algorithms::state::Frame MOUNTING_FRAME>
ImuWorldOrientationObserver<MOUNTING_FRAME>::ImuWorldOrientationObserver(
    const tap::communication::sensors::imu::ImuInterface& imu)
    : imu(imu)
{
}

template <aruwsrc::algorithms::state::Frame MOUNTING_FRAME>
DynamicOrientation ImuWorldOrientationObserver<MOUNTING_FRAME>::getOrientation() const
{
    return DynamicOrientation(
        imu.getRoll(),
        imu.getPitch(),
        imu.getYaw(),
        imu.getGx(),
        imu.getGy(),
        imu.getGz());
}
}  // namespace aruwsrc::control::turret

#endif  // IMU_WORLD_ORIENTATION_OBSERVER_HPP_