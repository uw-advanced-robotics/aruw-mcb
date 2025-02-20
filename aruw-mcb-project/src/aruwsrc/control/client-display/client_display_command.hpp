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

#ifndef CLIENT_DISPLAY_COMMAND_HPP_
#define CLIENT_DISPLAY_COMMAND_HPP_

#include <vector>

#include "tap/control/command.hpp"
#include "tap/drivers.hpp"

#include "indicators/hud_indicator.hpp"
#include "modm/processing/fiber.hpp"

#include "client_display_subsystem.hpp"

namespace aruwsrc::control::client_display
{
class ClientDisplaySubsystem;

/**
 * A command that controls what is displayed on the RoboMaster client's interactive HUD.
 *
 * @note Only a single ClientDisplayCommand should be instantiated. If more than one is
 * instantiated, this will lead to undefined behavior.
 */
class ClientDisplayCommand : public tap::control::Command, ::modm::Fiber<2048>
{
public:
    /**
     * Construct a ClientDisplayCommand.
     *
     * @param[in] drivers Global drivers instance.
     * @param[in] clientDisplay The client display subsystem associated with the command.
     * @param[in] hudIndicators A list of all the HUD indicators to be displayed.
     */
    ClientDisplayCommand(
        tap::Drivers &drivers,
        ClientDisplaySubsystem &clientDisplay,
        std::vector<HudIndicator *> &hudIndicators);

    const char *getName() const override { return "client display"; }

    void initialize() override;

    void execute() override;

    void end(bool) override {}

    bool isFinished() const override { return false; }

private:
    tap::Drivers &drivers;
    std::vector<HudIndicator *> &hudIndicators;

    bool restarting = true;

    float fps = 0.0f;
    uint32_t startTime = 0;

    bool run();
    void restartHud();
};
}  // namespace aruwsrc::control::client_display

#endif  // CLIENT_DISPLAY_COMMAND_HPP_
