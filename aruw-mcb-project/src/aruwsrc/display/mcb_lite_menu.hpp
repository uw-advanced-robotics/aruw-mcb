/*
 * Copyright (c) 2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef MCB_LITE_MENU_HPP_
#define MCB_LITE_MENU_HPP_

#include "tap/architecture/periodic_timer.hpp"
#include "tap/display/dummy_allocator.hpp"

#include "aruwsrc/communication/mcb-lite/mcb_lite.hpp"
#include "modm/ui/menu/abstract_menu.hpp"

namespace aruwsrc::display
{
/**
 * Menu that allows the user to see data coming from an MCB-lite
 */
class MCBLiteMenu : public modm::AbstractMenu<tap::display::DummyAllocator<modm::IAbstractView> >
{
public:
    static constexpr uint32_t DISPLAY_DRAW_PERIOD = 500;

    MCBLiteMenu(
        modm::ViewStack<tap::display::DummyAllocator<modm::IAbstractView> > *vs,
        aruwsrc::virtualMCB::MCBLite *mcbLite);

    void draw() override;

    void update() override;

    void shortButtonPress(modm::MenuButtons::Button button) override;

    bool hasChanged() override;

    static const char *getMenuName() { return "MCB Lite Menu"; }

private:
    static constexpr int MCB_LITE_MENU_ID = 14;

    aruwsrc::virtualMCB::MCBLite *mcbLite;

    tap::arch::PeriodicMilliTimer updatePeriodicTimer{DISPLAY_DRAW_PERIOD};
};

}  // namespace aruwsrc::display

#endif  // MCB_LITE_MENU_HPP_
