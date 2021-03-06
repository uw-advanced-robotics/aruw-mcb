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

#ifndef OLED_DISPLAY_HPP_
#define OLED_DISPLAY_HPP_

#include <modm/processing/protothread.hpp>
#include <modm/ui/menu/view_stack.hpp>

#include "aruwlib/architecture/periodic_timer.hpp"
#include "aruwlib/rm-dev-board-a/board.hpp"

#include "MainMenu.hpp"
#include "OledButtonHandler.hpp"
#include "SplashScreen.hpp"
#include "sh1106.hpp"
#include "util_macros.hpp"

namespace aruwlib
{
class Drivers;
namespace display
{
class OledDisplay : public ::modm::pt::Protothread
{
public:
    explicit OledDisplay(Drivers *drivers);
    DISALLOW_COPY_AND_ASSIGN(OledDisplay)
    mockable ~OledDisplay() = default;

    mockable void initialize();

    /**
     * Updates the display in a nonblocking fashion. This function uses protothreads
     * to call the sh1106's updateNonblocking function at a rate of 2 hz.
     *
     * @note This function uses protothreads (http://dunkels.com/adam/pt/).
     *      Local variables *do not* necessarily behave correctly and this
     *      function should be edited with care.
     */
    mockable bool updateDisplay();

    /**
     * Checks button state and updates the view stack responsible for determining what
     * should be displayed on the OLED.
     */
    mockable void updateMenu();

private:
    OledButtonHandler::Button prevButton = OledButtonHandler::NONE;

    Sh1106<
#ifndef PLATFORM_HOSTED
        Board::DisplaySpiMaster,
        Board::DisplayCommand,
        Board::DisplayReset,
#endif
        128,
        64,
        false>
        display;

    modm::ViewStack viewStack;

    OledButtonHandler buttonHandler;

    SplashScreen splashScreen;

    Drivers *drivers;

    aruwlib::arch::PeriodicMilliTimer displayThreadTimer{500};
};  // class OledDisplay
}  // namespace display
}  // namespace aruwlib

#endif  // OLED_DISPLAY_HPP_
