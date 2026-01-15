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

#include "indicators/hud_indicator.hpp"

#include "client_display_subsystem.hpp"

using namespace tap::control;

namespace aruwsrc::control::client_display
{
ClientDisplayCommand::ClientDisplayCommand(
    tap::Drivers &drivers,
    ClientDisplaySubsystem &clientDisplay,
    std::vector<indicators::HudIndicator *> &hudIndicators)
    : Command(),
      drivers(drivers),
      hudIndicators(hudIndicators),
      refSerialTransmitter(&drivers)
{
    addSubsystemRequirement(&clientDisplay);
    this->restartHud();
    numIndicators = hudIndicators.size();
}

void ClientDisplayCommand::initialize()
{
    // We cannot reset the thread from here because there might be locked
    // resources that we need to finish first.
    this->restarting = true;
}

void ClientDisplayCommand::restartHud()
{
    indicators::HudIndicator::resetGraphicNameGenerator();

    // Initialize all the HUD indicators
    for (auto &indicator : hudIndicators)
    {
        indicator->initialize();
    }

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

    PT_CALL(refSerialTransmitter.deleteGraphicLayer(RefSerialTransmitter::Tx::DELETE_ALL, 0));

    for (index = 0; index < numIndicators; index++)
    {
        PT_CALL(hudIndicators[index]->sendInitialGraphics());
    }

    // If we try to restart the hud, break out of the loop
    while (!this->restarting)
    {
        startTime = tap::arch::clock::getTimeMicroseconds();
        for (index = 0; index < numIndicators; index++)
        {
            PT_CALL(hudIndicators[index]->update());
        }

        // Calculate the time it took to update the HUD
        this->fps = 1e6 / (tap::arch::clock::getTimeMicroseconds() - startTime);

        PT_YIELD();
    }
    // Breaking out of the loop successfully calls this method,
    // allowing us to know that all execution is over.
    PT_END();
}

}  // namespace aruwsrc::control::client_display
