/*
 * Copyright (c) 2020-2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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
#ifndef ENGINEER_IMU_CALIBRATE_COMMAND_HPP_
#define ENGINEER_IMU_CALIBRATE_COMMAND_HPP_

#include <vector>

#include "tap/algorithms/odometry/odometry_2d_interface.hpp"
#include "tap/architecture/timeout.hpp"
#include "tap/control/command.hpp"

#include "aruwsrc/communication/can/turret_mcb_can_comm.hpp"
#include "aruwsrc/communication/mcb-lite/mcb_lite.hpp"
#include "aruwsrc/control/chassis/holonomic_chassis_subsystem.hpp"

namespace aruwsrc::engineer
{
class EngineerImuCalibrateCommand : public tap::control::Command
{
public:
    static constexpr float MAIN_LOOP_FREQUENCY = 500.0f;

    EngineerImuCalibrateCommand(
        tap::Drivers *drivers,
        aruwsrc::communication::mcb_lite::MCBLite *mcbLite,
        aruwsrc::control::chassis::HolonomicChassisSubsystem *chassis)
        : drivers(drivers),
          mcbLite(mcbLite)
    {
        addSubsystemRequirement(chassis);
    }

    const char *getName() const override { return "Engineer IMU Calibrate"; }
    bool isReady() override { return true; }
    void initialize() override {}
    int bill = 100;
    void execute() override
    {
        bill += 200;
        mcbLite->imu.requestCalibration();
    }
    void end(bool) override {}
    bool isFinished() const override { return false; }

private:
    tap::Drivers *drivers;
    aruwsrc::control::chassis::HolonomicChassisSubsystem *chassis;
    aruwsrc::communication::mcb_lite::MCBLite *mcbLite;
};
}  // namespace aruwsrc::engineer

#endif  // SENTRY_IMU_CALIBRATE_COMMAND_HPP_
