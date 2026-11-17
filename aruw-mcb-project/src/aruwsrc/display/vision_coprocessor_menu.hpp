/*
 * Copyright (c) 2026 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef VISION_COPROCESSOR_MENU_HPP_
#define VISION_COPROCESSOR_MENU_HPP_

#include "tap/architecture/periodic_timer.hpp"
#include "tap/display/dummy_allocator.hpp"
#include "tap/display/vertical_scroll_logic_handler.hpp"

#include "aruwsrc/communication/serial/vision_coprocessor.hpp"
#include "modm/ui/menu/abstract_menu.hpp"

namespace aruwsrc
{
class Drivers;
}

namespace aruwsrc::display
{
/**
 * Menu that shows the data exchanged between the mcb and the jetson.
 */
class VisionCoprocessorMenu
    : public modm::AbstractMenu<tap::display::DummyAllocator<modm::IAbstractView> >
{
public:
    /** Time between calls to `draw`, which will redraw the vision coprocessor menu. */
    static constexpr uint32_t DISPLAY_DRAW_PERIOD = 500;

    /** Max entries that can be displayed on the screen. */
    static constexpr uint8_t DISPLAY_MAX_ENTRIES = 8;

    VisionCoprocessorMenu(
        modm::ViewStack<tap::display::DummyAllocator<modm::IAbstractView> > *vs,
        tap::Drivers *drivers,
        communication::serial::VisionCoprocessor *visionCoprocessor);

    void draw() override;

    void update() override;

    void shortButtonPress(modm::MenuButtons::Button button) override;

    bool hasChanged() override;

    static const char *getMenuName() { return "Vision Coprocessor Menu"; }

private:
    static constexpr int VISION_COPROCESSOR_MENU_ID = 9;

    tap::Drivers *drivers;
    tap::display::VerticalScrollLogicHandler verticalScroll;
    communication::serial::VisionCoprocessor *visionCoprocessor;

    tap::arch::PeriodicMilliTimer updatePeriodicTimer{DISPLAY_DRAW_PERIOD};

    int8_t emitLines(modm::IOStream *stream, int8_t target) const;
};
}  // namespace aruwsrc::display

#endif  // VISION_COPROCESSOR_MENU_HPP_
