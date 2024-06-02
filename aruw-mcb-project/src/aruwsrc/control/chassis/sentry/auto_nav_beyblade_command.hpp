/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "tap/algorithms/ramp.hpp"
#include "tap/algorithms/smooth_pid.hpp"
#include "tap/control/command.hpp"
#include "tap/drivers.hpp"

#include "aruwsrc/algorithms/odometry/transformer_interface.hpp"
#include "aruwsrc/communication/serial/vision_coprocessor.hpp"
#include "aruwsrc/control/chassis/beyblade_command.hpp"
#include "aruwsrc/control/chassis/chassis_auto_nav_controller.hpp"
#include "aruwsrc/control/chassis/sentry/sentry_beyblade_config.hpp"
#include "aruwsrc/control/turret/turret_motor.hpp"
#include "aruwsrc/robot/control_operator_interface.hpp"

namespace aruwsrc::chassis
{
class HolonomicChassisSubsystem;

/**
 * A command that automatically rotates the chassis while maintaining turret angle
 */
class AutoNavBeybladeCommand : public tap::control::Command
{
public:
    AutoNavBeybladeCommand(
        tap::Drivers& drivers,
        HolonomicChassisSubsystem& chassis,
        aruwsrc::serial::VisionCoprocessor& visionCoprocessor,
        const aruwsrc::algorithms::transforms::TransformerInterface& transformerInterface,
        const aruwsrc::sentry::SentryBeybladeCommand::SentryBeybladeConfig& config,
        bool autoNavOnlyInGame = false);

    void initialize() override;

    void execute() override;

    void end(bool) override;

    bool isFinished() const override { return false; }

    inline void toggleBeyblade() { beybladeEnabled = !beybladeEnabled; };

    inline void toggleMovement() { movementEnabled = !movementEnabled; };

    const char* getName() const override { return "autonav beyblade"; }

private:
    tap::Drivers& drivers;
    HolonomicChassisSubsystem& chassis;
    aruwsrc::serial::VisionCoprocessor& visionCoprocessor;
    const aruwsrc::algorithms::transforms::TransformerInterface& transformerInterface;

    const aruwsrc::sentry::SentryBeybladeCommand::SentryBeybladeConfig& config;

    bool autoNavOnlyInGame;

    bool beybladeEnabled = false;
    bool movementEnabled = true;

    aruwsrc::chassis::ChassisAutoNavController autoNavController;

};  // class AutoNavBeybladeCommand

}  // namespace aruwsrc::chassis

#endif  // AUTO_NAV_BEYBLADE_COMMAND_HPP_
