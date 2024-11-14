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
    const launcher::FrictionWheelSubsystem &frictionWheelSubsystem,
    tap::control::setpoint::SetpointSubsystem &agitatorSubsystem,
    const control::turret::RobotTurretSubsystem &robotTurretSubsystem,
    const std::vector<tap::control::Command *> avoidanceCommands,
    const control::imu::ImuCalibrateCommand &imuCalibrateCommand,
    const aruwsrc::control::agitator::MultiShotCvCommandMapping *multiShotHandler,
    const aruwsrc::control::governor::CvOnTargetGovernor *cvOnTargetManager,
    algorithms::PlateHitTracker &plateHitTracker,
    TransformerInterface *transformer,
    const can::capbank::CapacitorBank *capBank)
    : Command(),
      drivers(drivers),
      visionCoprocessor(visionCoprocessor),
      commandScheduler(commandScheduler),
      refSerialTransmitter(&drivers),
      capBankIndicator(refSerialTransmitter, capBank),
      positionHudIndicators(
          drivers,
          visionCoprocessor,
          refSerialTransmitter,
          frictionWheelSubsystem,
          robotTurretSubsystem,
          multiShotHandler,
          cvOnTargetManager),
      ammoIndicator(refSerialTransmitter, drivers.refSerial),
      circleCrosshair(refSerialTransmitter),
      damageIndicator(plateHitTracker, robotTurretSubsystem, refSerialTransmitter),
      textHudIndicators(
          drivers,
          agitatorSubsystem,
          imuCalibrateCommand,
          avoidanceCommands,
          refSerialTransmitter),
      wallHack(visionCoprocessor, refSerialTransmitter, transformer)
{
    addSubsystemRequirement(&clientDisplay);
    this->restartHud();
}

void ClientDisplayCommand::initialize()
{
    // We cannot reset the thread from here because there might be locked
    // resources that we need to finish first.
    this->restarting = true;
}

void ClientDisplayCommand::restartHud()
{
    HudIndicator::resetGraphicNameGenerator();

    capBankIndicator.initialize();
    positionHudIndicators.initialize();
    ammoIndicator.initialize();
    circleCrosshair.initialize();
    damageIndicator.initialize();
    textHudIndicators.initialize();

    wallHack.initialize();

    // We can successfully restart the thread
    this->restarting = false;
}

void ClientDisplayCommand::execute() { run(); }

bool ClientDisplayCommand::run()
{
    // The thread has exited the loop, meaning that there are no locked resources
    if (!this->isRunning())
    {
        // Restart the thread
        restart();
        // Reset the HUD elements
        this->restartHud();
    }

    PT_BEGIN();

    PT_WAIT_UNTIL(drivers.refSerial.getRefSerialReceivingData());

    PT_CALL(capBankIndicator.sendInitialGraphics());
    PT_CALL(positionHudIndicators.sendInitialGraphics());
    PT_CALL(ammoIndicator.sendInitialGraphics());
    PT_CALL(circleCrosshair.sendInitialGraphics());
    PT_CALL(damageIndicator.sendInitialGraphics());
    PT_CALL(textHudIndicators.sendInitialGraphics());

    PT_CALL(wallHack.sendInitialGraphics());

    // If we try to restart the hud, break out of the loop
    while (!this->restarting)
    {
        startTime = tap::arch::clock::getTimeMilliseconds();
        PT_CALL(capBankIndicator.update());
        PT_CALL(positionHudIndicators.update());
        PT_CALL(ammoIndicator.update());
        PT_CALL(circleCrosshair.update());
        PT_CALL(damageIndicator.update());
        PT_CALL(textHudIndicators.update());

        PT_CALL(wallHack.update());
        // Calculate the time it took to update the HUD
        this->fps = 1000.0f / (tap::arch::clock::getTimeMilliseconds() - startTime) > 1000
                        ? this->fps
                        : 1000.0f / (tap::arch::clock::getTimeMilliseconds() - startTime);

        PT_YIELD();
    }
    // Breaking out of the loop successfully calls this method,
    // allowing us to know that all execution is over.
    PT_END();
}

}  // namespace aruwsrc::control::client_display
