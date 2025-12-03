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

#ifndef AUTO_NAV_BEYBLADE_COMMAND_HPP_
#define AUTO_NAV_BEYBLADE_COMMAND_HPP_

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/algorithms/ramp.hpp"
#include "tap/algorithms/smooth_pid.hpp"
#include "tap/architecture/clock.hpp"
#include "tap/communication/sensors/imu/mpu6500/mpu6500.hpp"
#include "tap/communication/serial/ref_serial_data.hpp"
#include "tap/communication/serial/remote.hpp"
#include "tap/control/command.hpp"
#include "tap/drivers.hpp"

#include "aruwsrc/algorithms/odometry/transforms/transformer_interface.hpp"
#include "aruwsrc/communication/serial/vision_coprocessor.hpp"
#include "aruwsrc/control/chassis/chassis_auto_nav_controller.hpp"
#include "aruwsrc/control/chassis/chassis_rel_drive.hpp"
#include "aruwsrc/control/chassis/holonomic_chassis_subsystem.hpp"
#include "aruwsrc/control/turret/turret_motor.hpp"
#include "aruwsrc/control/turret/turret_subsystem.hpp"
#include <aruwsrc/algorithms/odometry/chassis_cf_odometry.hpp>

namespace aruwsrc::control::chassis::sentry
{
class HolonomicChassisSubsystem;

/**
 * A command that automatically rotates the chassis while maintaining turret angle
 */
class AutoNavBeybladeCommand : public tap::control::Command
{
public:
    AutoNavBeybladeCommand(
        const tap::Drivers& drivers,
        chassis::HolonomicChassisSubsystem& chassis,
        aruwsrc::control::chassis::ChassisAutoNavController& autoNavController,
        aruwsrc::algorithms::odometry::ChassisCFOdometry& odometry,
        bool autoNavOnlyInGame = false);

    void initialize() override;

    void execute() override;

    void end(bool) override;

    bool isFinished() const override { return false; }

    inline void toggleBeyblade() { beybladeEnabled = false; };

    inline void toggleMovement() { movementEnabled = !movementEnabled; };

    const char* getName() const override { return "autonav beyblade"; }

private:
    const tap::Drivers& drivers;
    chassis::HolonomicChassisSubsystem& chassis;
    aruwsrc::control::chassis::ChassisAutoNavController& autoNavController;
    aruwsrc::algorithms::odometry::ChassisCFOdometry& odometry;

    bool autoNavOnlyInGame;

    bool beybladeEnabled = false;
    bool movementEnabled = true;

};  // class AutoNavBeybladeCommand

}  // namespace aruwsrc::control::chassis::sentry

#endif  // AUTO_NAV_BEYBLADE_COMMAND_HPP_
