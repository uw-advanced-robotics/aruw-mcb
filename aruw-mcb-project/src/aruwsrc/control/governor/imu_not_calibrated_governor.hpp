/*
 * Copyright (c) 2026-2026 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef IMU_NOT_CALIBRATED_GOVERNOR_HPP_
#define IMU_NOT_CALIBRATED_GOVERNOR_HPP_

#include "tap/communication/sensors/imu/abstract_imu.hpp"
#include "tap/control/governor/command_governor_interface.hpp"
#include "tap/drivers.hpp"

namespace aruwsrc::control::governor
{
/**
 * A governor that allows a Command to run if an imu isn't calibrated or trying to.
 */
class ImuNotCalibratedGovernor : public tap::control::governor::CommandGovernorInterface
{
public:
    ImuNotCalibratedGovernor(
        tap::Drivers* drivers,
        tap::communication::sensors::imu::AbstractIMU& imu)
        : drivers(drivers),
          imu(imu)
    {
    }

    mockable void setGovernorEnabled(bool enabled) { this->enabled = enabled; }

    mockable bool isGoverEnabled() const { return this->enabled; }

    bool isReady() final_mockable
    {
        return imu.getImuState() ==
               tap::communication::sensors::imu::ImuInterface::ImuState::IMU_NOT_CALIBRATED;
    }

    bool isFinished() final_mockable
    {
        // Once started, command is allowed to run to completion.
        return false;
    }

private:
    bool enabled = true;
    tap::Drivers* drivers;
    tap::communication::sensors::imu::AbstractIMU& imu;
};
}  // namespace aruwsrc::control::governor

#endif  // IMU_NOT_CALIBRATED_GOVERNOR_HPP_