/*
 * Copyright (c) 2020-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "client_display_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/errors/create_errors.hpp"

#include "aruwsrc/drivers.hpp"

#include "client_display_subsystem.hpp"
#include "hud_indicator.hpp"

using namespace tap::control;

namespace aruwsrc::control::client_display
{
ClientDisplayCommand::ClientDisplayCommand(
    aruwsrc::Drivers *drivers,
    ClientDisplaySubsystem *clientDisplay,
    const TurretMCBHopperSubsystem *hopperSubsystem,
    const launcher::FrictionWheelSubsystem &frictionWheelSubsystem,
    agitator::AgitatorSubsystem &agitatorSubsystem,
    const control::turret::TurretSubsystem &turretSubsystem,
    const control::imu::ImuCalibrateCommand &imuCalibrateCommand,
    const chassis::BeybladeCommand *chassisBeybladeCmd,
    const chassis::ChassisAutorotateCommand *chassisAutorotateCmd,
    const chassis::ChassisImuDriveCommand *chassisImuDriveCommand)
    : Command(),
      drivers(drivers),
      booleanHudIndicators(
          drivers,
          hopperSubsystem,
          frictionWheelSubsystem,
          agitatorSubsystem,
          imuCalibrateCommand),
      chassisOrientationIndicator(drivers, turretSubsystem),
      positionHudIndicators(
          drivers,
          hopperSubsystem,
          frictionWheelSubsystem,
          chassisBeybladeCmd,
          chassisAutorotateCmd,
          chassisImuDriveCommand),
      reticleIndicator(drivers),
      turretAnglesIndicator(drivers, turretSubsystem)
{
    modm_assert(drivers != nullptr, "ClientDisplayCommand", "drivers nullptr");
    addSubsystemRequirement(clientDisplay);
}

void ClientDisplayCommand::initialize()
{
    HudIndicator::resetGraphicNameGenerator();
    restart();  // restart protothread
    booleanHudIndicators.initialize();
    chassisOrientationIndicator.initialize();
    positionHudIndicators.initialize();
    reticleIndicator.initialize();
    turretAnglesIndicator.initialize();
}

void ClientDisplayCommand::execute() { run(); }

bool ClientDisplayCommand::run()
{
    PT_BEGIN();

    PT_WAIT_UNTIL(drivers->refSerial.getRefSerialReceivingData());

    PT_CALL(booleanHudIndicators.sendInitialGraphics());
    PT_CALL(chassisOrientationIndicator.sendInitialGraphics());
    PT_CALL(positionHudIndicators.sendInitialGraphics());
    PT_CALL(reticleIndicator.sendInitialGraphics());
    PT_CALL(turretAnglesIndicator.sendInitialGraphics());

    while (true)
    {
        PT_CALL(booleanHudIndicators.update());
        PT_CALL(chassisOrientationIndicator.update());
        PT_CALL(positionHudIndicators.update());
        PT_CALL(reticleIndicator.update());
        PT_CALL(turretAnglesIndicator.update());
        PT_YIELD();
    }
    PT_END();
}

}  // namespace aruwsrc::control::client_display
