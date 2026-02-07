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

#ifndef RTT_MENU_HPP_
#define RTT_MENU_HPP_

#include <functional>

#include "tap/display/dummy_allocator.hpp"

#include "aruwsrc/communication/rtt/rtt_connection_state.hpp"
#include "aruwsrc/communication/rtt/rtt_telemetry.hpp"
#include "modm/ui/menu/abstract_menu.hpp"

namespace aruwsrc
{
class Drivers;
}

namespace aruwsrc
{
namespace display
{
class RttMenu : public modm::AbstractMenu<tap::display::DummyAllocator<modm::IAbstractView>>
{
public:
    // Time between calls to `draw`, which will redraw the rtt status menu.
    static constexpr uint32_t DISPLAY_DRAW_PERIOD = 500;

    RttMenu(
        modm::ViewStack<tap::display::DummyAllocator<modm::IAbstractView>> *vs,
        aruwsrc::communication::rtt::RttTelemetry *telemetry);

    void draw() override;

    void update() override;

    void shortButtonPress(modm::MenuButtons::Button button) override;

    bool hasChanged() override;

    static const char *getMenuName() { return "RTT Menu"; }

private:
    static constexpr int RTT_MENU_ID = 16;

    static constexpr int TURRET_MCB_MENU_ID = 12;

    aruwsrc::communication::rtt::RttTelemetry *telemetry;

    tap::arch::PeriodicMilliTimer updatePeriodicTimer{DISPLAY_DRAW_PERIOD};
};  // class RttMenu
}  // namespace display
}  // namespace aruwsrc

#endif  // RTT_MENU_HPP_
