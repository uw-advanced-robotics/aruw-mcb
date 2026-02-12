/*
 * Copyright (c) 2024-2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef LIMIT_SWITCH_MENU_HPP_
#define LIMIT_SWTICH_MENU_HPP_

#include "tap/communication/can/can_bus.hpp"
#include "tap/drivers.hpp"
#include "tap/communication/gpio/digital.hpp"
#include "modm/ui/menu/abstract_menu.hpp"

#include "tap/display/dummy_allocator.hpp"
#include "tap/display/vertical_scroll_logic_handler.hpp"

using namespace tap::gpio;

namespace aruwsrc
{
class Drivers;
}  // namespace aruwsrc

namespace aruwsrc::display
{
class LimitSwitchMenu : public modm::AbstractMenu<tap::display::DummyAllocator<modm::IAbstractView> >
{
public:
    LimitSwitchMenu(
        modm::ViewStack<tap::display::DummyAllocator<modm::IAbstractView> > *stack,
        tap::Drivers *drivers);

    void draw() override;

    void update() override;

    bool hasChanged() override;

    void shortButtonPress(modm::MenuButtons::Button button) override;

    static const char *getMenuName() { return "Limit Switch Menu"; }

private:
    static constexpr int LIMIT_SWITCH_MENU_ID = 16;
    
    tap::Drivers *drivers;
    
    static constexpr std::array<std::string_view, 5> InputPinNames{
        "B", "C", "D", "T", "Button"
    };

    void drawLimitSwitch(Digital::InputPin pin);

    // void setPinValue(Digital::InputPin pin, int val);
    
    std::map<tap::gpio::Digital::InputPin, int> pins = {  //theres prolly a way to not hardcode it i assume but idk rn
        {tap::gpio::Digital::InputPin::B, -1},
        {tap::gpio::Digital::InputPin::C, -1},
        {tap::gpio::Digital::InputPin::D, -1}, 
        {tap::gpio::Digital::InputPin::T, -1},
        {tap::gpio::Digital::InputPin::Button, -1}
    };

    // is this right
    // static constexpr std::array<std::pair<Digital::InputPin, int>, 5> pins = {{
    //     {Digital::InputPin::B, -1},
    //     {Digital::InputPin::C, -1},
    //     {Digital::InputPin::D, -1}, 
    //     {Digital::InputPin::T, -1},
    //     {Digital::InputPin::Button, -1}
    // }};
    
};
}  // namespace display
  

#endif  // LIMIT_SWITCH_MENU_HPP_
