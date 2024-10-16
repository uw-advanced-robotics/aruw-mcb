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

#ifndef IMU_CALIBRATE_DONE_GOVERNOR_HPP_
#define IMU_CALIBRATE_DONE_GOVERNOR_HPP_

#include "aruwsrc/control/imu/imu_calibrate_command.hpp"
#include "tap/control/governor/command_governor_interface.hpp"
#include "tap/drivers.hpp"

namespace aruwsrc::control::governor
{


/**
 * A governor that allows a Command to run when a IMUCalibrateCommand is not scheduled
 */
class IMUCalibrateDoneGovernor : public tap::control::governor::CommandGovernorInterface
{
public:
    IMUCalibrateDoneGovernor(
        tap::Drivers *drivers,
        aruwsrc::control::imu::ImuCalibrateCommand& imuCalibrateCommand)
        : drivers(drivers),
            imuCalibrateCommand(imuCalibrateCommand)
    {
    }

    mockable void setGovernorEnabled(bool enabled) { this->enabled = enabled; }

    mockable bool isGoverEnabled() const { return this->enabled; }

    bool isReady() final_mockable
    {
        return !(drivers->commandScheduler.isCommandScheduled(&imuCalibrateCommand));

    }

    bool isFinished() final_mockable
    {
        // Once started, command is allowed to run to completion.
        return false;
    }

private:
    bool enabled = true;
    tap::Drivers *drivers;
    aruwsrc::control::imu::ImuCalibrateCommand& imuCalibrateCommand;
    
};
}  // namespace aruwsrc::control::governor

#endif  // IMU_CALIBRATE_DONE_GOVERNOR_HPP_