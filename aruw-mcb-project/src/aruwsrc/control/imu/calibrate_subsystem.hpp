/*
 * Copyright (c) 2022-2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef CALIBRATE_SUBSYSTEM_HPP_
#define CALIBRATE_SUBSYSTEM_HPP_

#include "tap/control/subsystem.hpp"

#include "aruwsrc/control/imu/imu_calibrate_command.hpp"
#include "aruwsrc/robot/control_operator_interface.hpp"

namespace aruwsrc::control::imu
{
class CalibrateSubsystem : public tap::control::Subsystem
{
public:
    CalibrateSubsystem(
        tap::Drivers* drivers,
        ImuCalibrateCommand& imuCalibrateCommand,
        ControlOperatorInterface& controlOperatorInterface);

    void calibrateImu();

    void refresh() override;

    const char* getName() override { return "Calibrate"; }

private:
    ImuCalibrateCommand& imuCalibrateCommand;
    ControlOperatorInterface& controlOperatorInterface;
};
}  // namespace aruwsrc::control::imu
#endif