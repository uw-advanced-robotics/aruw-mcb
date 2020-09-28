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

#include <modm/math/filter/debounce.hpp>
#include <modm/ui/menu/view_stack.hpp>

#include "aruwlib/rm-dev-board-a/board.hpp"

#include "MainMenu.hpp"
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
    static constexpr int BUTTON_DEBOUNCE_SAMPLES = 10;
    static constexpr int DEBOUNCE_ADC_PRESSED_RANGE = 100;
    static constexpr int MIN_ADC_BEFORE_BUTTON_IN_IDLE_STATE = 3500;
    static constexpr int DOWN_ADC_VAL = 3300;
    static constexpr int UP_ADC_VAL = 2500;
    static constexpr int LEFT_ADC_VAL = 900;
    static constexpr int RIGHT_ADC_VAL = 1700;
    static constexpr int OK_ADC_VAL = 0;

    void handleButtonStatus();
    bool buttonIsIdle;

    Sh1106<
#ifndef ENV_SIMULATOR
        Board::DisplaySpiMaster,
        Board::DisplayCommand,
        Board::DisplayReset,
#endif
        128,
        64,
        false>
        display;

    modm::ViewStack vs;

    MainMenu mainMenu;

    modm::filter::Debounce<int> downButtonPressed;
    modm::filter::Debounce<int> upButtonPressed;
    modm::filter::Debounce<int> leftButtonPressed;
    modm::filter::Debounce<int> rightButtonPressed;
    modm::filter::Debounce<int> okButtonPressed;

    Drivers *drivers;
};  // class OledDisplay
}  // namespace display
}  // namespace aruwlib

#endif  // OLED_DISPLAY_HPP_
