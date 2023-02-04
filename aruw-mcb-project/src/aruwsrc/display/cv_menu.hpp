/*
 * Copyright (c) 2021-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef CV_MENU_HPP_
#define CV_MENU_HPP_

#include "tap/architecture/periodic_timer.hpp"
#include "tap/display/dummy_allocator.hpp"
#include "tap/display/vertical_scroll_logic_handler.hpp"

#include "modm/ui/menu/abstract_menu.hpp"

#include "aruwsrc/communication/serial/vision_coprocessor.hpp"

namespace aruwsrc
{
class Drivers;
}

namespace aruwsrc::display
{
/**
 * Menu that allows a user to interact with the vision coprocessor. Currently, lets the user turn
 * off the vision coprocessor or reboot it. Also shows the current status of the vision coprocessor.
 */
class CVMenu : public modm::AbstractMenu<tap::display::DummyAllocator<modm::IAbstractView> >
{
public:
    /** Time between calls to `draw`, which will redraw the referee serial menu. */
    static constexpr uint32_t DISPLAY_DRAW_PERIOD = 500;

    /** Max entries that can be displayed on the screen. */
    static constexpr uint8_t DISPLAY_MAX_ENTRIES = 8;

    /**
     * @param[in] vs `ViewStack` that this menu is sitting on top of.
     * @param[in] drivers A pointer to the global drivers object.
     * @param[in] visionCoprocessor A pointer to the global visionCoprocessor object.
     */
    CVMenu(
        modm::ViewStack<tap::display::DummyAllocator<modm::IAbstractView> > *vs,
        tap::Drivers *drivers,
        serial::VisionCoprocessor *visionCoprocessor);

    void draw() override;

    void update() override;

    void shortButtonPress(modm::MenuButtons::Button button) override;

    bool hasChanged() override;

    static const char *getMenuName() { return "CV Menu"; }

private:
    using PrintCVMenuLineFn = void (CVMenu::*)(modm::IOStream &);
    using UpdateCVStateFn = void (CVMenu::*)();

    static constexpr int CV_MENU_ID = 8;

    tap::Drivers *drivers;
    tap::display::VerticalScrollLogicHandler verticalScroll;
    serial::VisionCoprocessor *visionCoprocessor;

    tap::arch::PeriodicMilliTimer updatePeriodicTimer{DISPLAY_DRAW_PERIOD};

    void drawShutdownCV(modm::IOStream &stream);
    void shutdownCV();
    void drawRebootCV(modm::IOStream &stream);
    void rebootCV();
    void drawCVOnline(modm::IOStream &stream);
    void pass() {}

    static constexpr std::tuple<PrintCVMenuLineFn, UpdateCVStateFn> CV_MENU_UPDATE_FNS[] = {
        {&CVMenu::drawShutdownCV, &CVMenu::shutdownCV},
        {&CVMenu::drawRebootCV, &CVMenu::rebootCV},
        {&CVMenu::drawCVOnline, &CVMenu::pass},
    };
};
}  // namespace aruwsrc::display

#endif  // CV_MENU_HPP_
