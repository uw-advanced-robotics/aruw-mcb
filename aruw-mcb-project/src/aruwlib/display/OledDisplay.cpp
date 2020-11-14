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

#include "OledDisplay.hpp"

#include "aruwlib/Drivers.hpp"

using namespace modm::literals;

namespace aruwlib
{
namespace display
{
void *OledDisplay::readCin(void *vargs)
{
    OledDisplay *display = reinterpret_cast<OledDisplay *>(vargs);

    while (true)
    {
        char c;
        std::cin >> c;
        pthread_mutex_lock(&display->cinDataMutex);
        display->currRequest = c;
        pthread_mutex_unlock(&display->cinDataMutex);
    }
}

OledDisplay::OledDisplay(Drivers *drivers)
    : display(),
      viewStack(&display),
      buttonHandler(drivers),
      mainMenu(&viewStack, 0, drivers),
      drivers(drivers)
{
}

void OledDisplay::initialize()
{
#ifndef PLATFORM_HOSTED
    Board::DisplaySpiMaster::
        connect<Board::DisplayMiso::Miso, Board::DisplayMosi::Mosi, Board::DisplaySck::Sck>();

    // SPI1 is on ABP2 which is at 90MHz; use prescaler 64 to get ~fastest baud rate below 1mHz max
    // 90MHz/64=~14MHz
    Board::DisplaySpiMaster::initialize<Board::SystemClock, 1406250_Hz>();
#else
    pthread_mutex_init(&cinDataMutex, nullptr);
    pthread_create(&cinRxThread, nullptr, &readCin, reinterpret_cast<void *>(this));
#endif

    display.initializeBlocking();
    display.setFont(modm::font::ScriptoNarrow);

    mainMenu.initialize();

    viewStack.push(&mainMenu);
}

void OledDisplay::update()
{
#ifdef PLATFORM_HOSTED
    OledButtonHandler::Button buttonPressed;
    pthread_mutex_lock(&cinDataMutex);
    if (currRequest == DOWN)
    {
        buttonPressed = OledButtonHandler::DOWN;
    }
    else if (currRequest == UP)
    {
        buttonPressed = OledButtonHandler::UP;
    }
    else if (currRequest == LEFT)
    {
        buttonPressed = OledButtonHandler::LEFT;
    }
    else if (currRequest == RIGHT)
    {
        buttonPressed = OledButtonHandler::RIGHT;
    }
    else if (currRequest == OK)
    {
        buttonPressed = OledButtonHandler::OK;
    }
    else
    {
        buttonPressed = OledButtonHandler::NONE;
    }
    pthread_mutex_unlock(&cinDataMutex);
#else
    OledButtonHandler::Button buttonPressed = buttonHandler.getCurrentButtonState();
#endif
    if (buttonPressed != OledButtonHandler::NONE && buttonPressed != prevButton)
    {
        // Seperate from above for ease of readability.
        // For now the main menu should never be removed from the stack (which is what the left
        // button does in a StandardMenu).
        if (buttonPressed != OledButtonHandler::LEFT || viewStack.get() != &mainMenu)
        {
            viewStack.shortButtonPress(static_cast<modm::MenuButtons::Button>(buttonPressed));
        }
    }
    prevButton = buttonPressed;

    viewStack.update();
}
}  // namespace display
}  // namespace aruwlib
