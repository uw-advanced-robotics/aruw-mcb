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
#include "tap/drivers.hpp"
#include "tap/errors/create_errors.hpp"

#include "aruwsrc/communication/serial/vision_coprocessor.hpp"

#include "client_display_subsystem.hpp"
#include "hud_indicator.hpp"

using namespace tap::control;

namespace aruwsrc::control::client_display
{
ClientDisplayCommand::ClientDisplayCommand(
    tap::Drivers &drivers,
    tap::control::CommandScheduler &commandScheduler,
    aruwsrc::serial::VisionCoprocessor &visionCoprocessor,
    ClientDisplaySubsystem &clientDisplay,
    const TurretMCBHopperSubsystem *hopperSubsystem,
    const launcher::FrictionWheelSubsystem &frictionWheelSubsystem,
    tap::control::setpoint::SetpointSubsystem &agitatorSubsystem,
    const control::turret::RobotTurretSubsystem &robotTurretSubsystem,
    const control::imu::ImuCalibrateCommand &imuCalibrateCommand,
    const aruwsrc::control::agitator::MultiShotCvCommandMapping *multiShotHandler,
    const aruwsrc::control::governor::CvOnTargetGovernor *cvOnTargetManager,
    const chassis::BeybladeCommand *chassisBeybladeCmd,
    const chassis::ChassisAutorotateCommand *chassisAutorotateCmd,
    const chassis::ChassisImuDriveCommand *chassisImuDriveCommand,
    const aruwsrc::control::BarrelSwitchCommand *barrelSwitchCommand,
    const aruwsrc::communication::serial::SentryResponseHandler &sentryResponseHandler,
    const aruwsrc::chassis::BeybladeCommand& beybladeCommand)
    : Command(),
      drivers(drivers),
      visionCoprocessor(visionCoprocessor),
      commandScheduler(commandScheduler),
      refSerialTransmitter(&drivers),
      booleanHudIndicators(
          commandScheduler,
          refSerialTransmitter,
          hopperSubsystem,
          frictionWheelSubsystem,
          agitatorSubsystem,
          imuCalibrateCommand,
          barrelSwitchCommand,
          sentryResponseHandler,
          beybladeCommand),
      chassisOrientationIndicator(drivers, refSerialTransmitter, robotTurretSubsystem),
      positionHudIndicators(
          drivers,
          visionCoprocessor,
          refSerialTransmitter,
          hopperSubsystem,
          frictionWheelSubsystem,
          robotTurretSubsystem,
          multiShotHandler,
          cvOnTargetManager,
          chassisBeybladeCmd,
          chassisAutorotateCmd,
          chassisImuDriveCommand,
          &sentryResponseHandler),
      reticleIndicator(drivers, refSerialTransmitter),
      visionHudIndicators(visionCoprocessor, refSerialTransmitter),
      holdFireIndicator(sentryResponseHandler, refSerialTransmitter),
      sentryResponseHandler(sentryResponseHandler)
{
    addSubsystemRequirement(&clientDisplay);
}

void ClientDisplayCommand::initialize()
{
    HudIndicator::resetGraphicNameGenerator();
    restart();  // restart protothread
    booleanHudIndicators.initialize();
    // chassisOrientationIndicator.initialize();
    positionHudIndicators.initialize();
    reticleIndicator.initialize();
    visionHudIndicators.initialize();
    holdFireIndicator.initialize();
}

void ClientDisplayCommand::execute() { run(); }

bool ClientDisplayCommand::run()
{
    PT_BEGIN();

    PT_WAIT_UNTIL(drivers.refSerial.getRefSerialReceivingData());

    PT_CALL(booleanHudIndicators.sendInitialGraphics());
    // PT_CALL(chassisOrientationIndicator.sendInitialGraphics());
    PT_CALL(positionHudIndicators.sendInitialGraphics());
    PT_CALL(reticleIndicator.sendInitialGraphics());
    PT_CALL(visionHudIndicators.sendInitialGraphics());
    PT_CALL(holdFireIndicator.sendInitialGraphics());

    while (true) {
        counter++;
        
        if (counter%6 == 0) {
            PT_CALL(booleanHudIndicators.update());
        } else if (counter%6 == 1 ) {
            // PT_CALL(chassisOrientationIndicator.update());
        } else if (counter%6 == 2) {
            PT_CALL(positionHudIndicators.update());
        } else if (counter%6 == 3) {
            PT_CALL(reticleIndicator.update());
        } else if (counter%6 == 4) {
            PT_CALL(visionHudIndicators.update());
        } else if (counter%6 == 5) {
            PT_CALL(holdFireIndicator.update());
        }

        PT_YIELD();
    }
    PT_END();
}

}  // namespace aruwsrc::control::client_display
