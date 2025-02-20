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

#include "client_display_subsystem.hpp"
#include "indicators/hud_indicator.hpp"

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
      Fiber([this] { run(); }),
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
      visionTargetIndicator(visionCoprocessor, refSerialTransmitter, transformer->getWorldToVTM()),
      imageIndicator(refSerialTransmitter)
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
    visionTargetIndicator.initialize();

    imageIndicator.initialize();

    // We can successfully restart the thread
    this->restarting = false;
}

void ClientDisplayCommand::execute() {}

bool ClientDisplayCommand::run()
{
    PT_WAIT_UNTIL(drivers.refSerial.getRefSerialReceivingData());

    while (true)
    {
        // Reset the HUD elements
        this->restartHud();

        capBankIndicator.sendInitialGraphics();
        positionHudIndicators.sendInitialGraphics();
        ammoIndicator.sendInitialGraphics();
        circleCrosshair.sendInitialGraphics();
        damageIndicator.sendInitialGraphics();
        textHudIndicators.sendInitialGraphics();
        visionTargetIndicator.sendInitialGraphics();
        imageIndicator.sendInitialGraphics();

        // If we try to restart the hud, break out of the loop
        while (!this->restarting)
        {
            startTime = tap::arch::clock::getTimeMicroseconds();
            capBankIndicator.update();
            positionHudIndicators.update();
            ammoIndicator.update();
            circleCrosshair.update();
            damageIndicator.update();
            textHudIndicators.update();
            visionTargetIndicator.update();
            imageIndicator.update();

            // Calculate the time it took to update the HUD
            this->fps = 1e6 / (tap::arch::clock::getTimeMicroseconds() - startTime);

            PT_YIELD();
        }
    }

    return false;
}

}  // namespace aruwsrc::control::client_display
