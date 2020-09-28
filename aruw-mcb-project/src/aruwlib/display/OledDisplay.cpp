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
OledDisplay::OledDisplay(Drivers *drivers)
    : buttonIsIdle(true),
      display(),
      vs(&display),
      mainMenu(&vs, 0, drivers),
      downButtonPressed(BUTTON_DEBOUNCE_SAMPLES),
      upButtonPressed(BUTTON_DEBOUNCE_SAMPLES),
      leftButtonPressed(BUTTON_DEBOUNCE_SAMPLES),
      rightButtonPressed(BUTTON_DEBOUNCE_SAMPLES),
      okButtonPressed(BUTTON_DEBOUNCE_SAMPLES),
      drivers(drivers)
{
}

void OledDisplay::initialize()
{
#ifndef ENV_SIMULATOR
    Board::DisplaySpiMaster::
        connect<Board::DisplayMiso::Miso, Board::DisplayMosi::Mosi, Board::DisplaySck::Sck>();

    // SPI1 is on ABP2 which is at 90MHz; use prescaler 64 to get ~fastest baud rate below 1mHz max
    // 90MHz/64=~14MHz
    Board::DisplaySpiMaster::initialize<Board::SystemClock, 1406250_Hz>();
#endif

    display.initializeBlocking();
    display.setFont(modm::font::ScriptoNarrow);

    mainMenu.initialize();

    vs.push(&mainMenu);
}

void OledDisplay::handleButtonStatus()
{
    int buttonADC = drivers->analog.read(gpio::Analog::Pin::OLED_BUTTON);

    downButtonPressed.update(abs(buttonADC - DOWN_ADC_VAL) < DEBOUNCE_ADC_PRESSED_RANGE);
    upButtonPressed.update(abs(buttonADC - UP_ADC_VAL) < DEBOUNCE_ADC_PRESSED_RANGE);
    leftButtonPressed.update(abs(buttonADC - LEFT_ADC_VAL) < DEBOUNCE_ADC_PRESSED_RANGE);
    rightButtonPressed.update(abs(buttonADC - RIGHT_ADC_VAL) < DEBOUNCE_ADC_PRESSED_RANGE);
    okButtonPressed.update(abs(buttonADC - OK_ADC_VAL) < DEBOUNCE_ADC_PRESSED_RANGE);

    if (buttonADC > MIN_ADC_BEFORE_BUTTON_IN_IDLE_STATE && !downButtonPressed.getValue() &&
        !upButtonPressed.getValue() && !leftButtonPressed.getValue() &&
        !rightButtonPressed.getValue() && !okButtonPressed.getValue())
    {
        buttonIsIdle = true;
        return;
    }
    if (buttonIsIdle)
    {
        if (downButtonPressed.getValue())
        {
            buttonIsIdle = false;
            vs.shortButtonPress(modm::MenuButtons::DOWN);
        }
        else if (upButtonPressed.getValue())
        {
            buttonIsIdle = false;
            vs.shortButtonPress(modm::MenuButtons::UP);
        }
        else if (rightButtonPressed.getValue())
        {
            buttonIsIdle = false;
            vs.shortButtonPress(modm::MenuButtons::RIGHT);
        }
        else if (leftButtonPressed.getValue() && vs.get() != &mainMenu)
        {
            // For now the main menu should never be removed from the stack (which is what the left
            // button does in a StandardMenu).
            buttonIsIdle = false;
            vs.shortButtonPress(modm::MenuButtons::LEFT);
        }
        else if (okButtonPressed.getValue())
        {
            buttonIsIdle = false;
            vs.shortButtonPress(modm::MenuButtons::OK);
        }
    }
}

void OledDisplay::update()
{
    handleButtonStatus();
    vs.update();
}
}  // namespace display
}  // namespace aruwlib
