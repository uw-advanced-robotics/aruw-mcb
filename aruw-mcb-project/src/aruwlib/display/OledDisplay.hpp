/*
 * Copyright (c) 2020 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include <modm/ui/menu/view_stack.hpp>

#ifdef PLATFORM_HOSTED
#include <pthread.h>
#endif

#include "aruwlib/rm-dev-board-a/board.hpp"

#include "MainMenu.hpp"
#include "OledButtonHandler.hpp"
#include "mock_macros.hpp"
#include "sh1106.hpp"

namespace aruwlib
{
class Drivers;
namespace display
{
class OledDisplay
{
public:
    explicit OledDisplay(Drivers *drivers);
    OledDisplay(const OledDisplay &) = delete;
    OledDisplay &operator=(const OledDisplay &) = delete;
    mockable ~OledDisplay() = default;

    mockable void initialize();

    mockable void update();

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

#ifdef PLATFORM_HOSTED
    static constexpr char DOWN = 's';
    static constexpr char UP = 'w';
    static constexpr char LEFT = 'a';
    static constexpr char RIGHT = 'd';
    static constexpr char OK = 'q';

    pthread_mutex_t cinDataMutex;
    pthread_t cinRxThread;
    char currRequest = '\0';

    static void *readCin(void *vargs);
#endif

    modm::ViewStack viewStack;

    OledButtonHandler buttonHandler;

    MainMenu mainMenu;

    Drivers *drivers;
};  // class OledDisplay
}  // namespace display
}  // namespace aruwlib

#endif  // OLED_DISPLAY_HPP_
