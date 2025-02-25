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
    std::vector<HudIndicator *> &hudIndicators)
    : Command(),
      Fiber([this] { run(); }),
      drivers(drivers),
      hudIndicators(hudIndicators)
{
    addSubsystemRequirement(&clientDisplay);
    this->stack_watermark();
    numIndicators = hudIndicators.size();
}

void ClientDisplayCommand::initialize()
{
    this->restarting = true;
}

void ClientDisplayCommand::restartHud()
{
    HudIndicator::resetGraphicNameGenerator();

    // Initialize all the HUD indicators
    for (auto &indicator : hudIndicators)
    {
        indicator->initialize();
    }

    // We can successfully restart the thread
    this->restarting = false;
}

void ClientDisplayCommand::execute()
{
    if (!this->isRunning()) this->start();
}

bool ClientDisplayCommand::run()
{
    // while (!drivers.refSerial.getRefSerialReceivingData()) modm::this_fiber::yield();

    while (true)
    {
        // Reset the HUD elements
        this->restartHud();

        // Have each indicator send their initial graphics
        for (auto &indicator : hudIndicators)
        {
            indicator->sendInitialGraphics();
        }

        // If we try to restart the hud, break out of the loop
        while (!this->restarting)
        {
            startTime = tap::arch::clock::getTimeMicroseconds();

            // Update all the HUD indicators
            for (auto &indicator : hudIndicators)
            {
                indicator->update();
            }

            // Calculate the time it took to update the HUD
            this->fps = 1e6 / (tap::arch::clock::getTimeMicroseconds() - startTime);

            modm::this_fiber::yield();
        }
    }

    return false;
}

}  // namespace aruwsrc::control::client_display
