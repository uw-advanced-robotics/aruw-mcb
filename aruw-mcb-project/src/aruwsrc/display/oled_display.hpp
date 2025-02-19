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

#include "tap/board/board.hpp"
#include "tap/display/oled_button_handler.hpp"
#include "tap/display/sh1106.hpp"
#include "tap/display/sh1107.hpp"
#include "tap/util_macros.hpp"

#include "aruwsrc/communication/mcb-lite/mcb_lite.hpp"
#include "modm/platform.hpp"
#include "modm/processing/fiber.hpp"
#include "modm/ui/menu/view_stack.hpp"

#include "splash_screen.hpp"

namespace aruwsrc
{
class Drivers;
}

namespace aruwsrc
{
namespace display
{
class OledDisplay : public ::modm::Fiber<1024>
{
public:
    explicit OledDisplay(
        tap::Drivers *drivers,
        serial::VisionCoprocessor *visionCoprocessor,
        can::TurretMCBCanComm *turretMCBCanCommBus1,
        can::TurretMCBCanComm *turretMCBCanCommBus2,
        aruwsrc::virtualMCB::MCBLite *mcbLite1,
        aruwsrc::virtualMCB::MCBLite *mcbLite2,
        can::capbank::CapacitorBank *capacitorBank = nullptr);
    DISALLOW_COPY_AND_ASSIGN(OledDisplay)
    mockable ~OledDisplay() = default;

    mockable void initialize();

    /**
     * Updates the display in a nonblocking fashion. This function uses protothreads
     * to call the Sh1107's updateNonblocking function at a rate of 2 hz.
     *
     * @note This function uses protothreads (http://dunkels.com/adam/pt/).
     *      Local variables *do not* necessarily behave correctly and this
     *      function should be edited with care.
     */
    inline void run();

    /**
     * Checks button state and updates the view stack responsible for determining what
     * should be displayed on the OLED.
     */
    mockable void updateMenu();

private:
#if defined(OLD_ROBOTS)
    tap::display::OledButtonHandler::Button prevButton = tap::display::OledButtonHandler::NONE;
    tap::display::Sh1106<
#ifndef PLATFORM_HOSTED
        Board::DisplaySpiMaster,
        Board::DisplayCommand,
        Board::DisplayReset,
#endif
        128,
        64,
        false>
        display;
#else
    tap::display::OledButtonHandler::Button prevButton = tap::display::OledButtonHandler::NONE;
    tap::display::Sh1107<
#ifndef PLATFORM_HOSTED
        Board::DisplaySpiMaster,
        Board::DisplayCommand,
        Board::DisplayReset,
#endif
        128,
        128,
        true,
        true>
        display;
#endif

    modm::ViewStack<tap::display::DummyAllocator<modm::IAbstractView> > viewStack;

    tap::display::OledButtonHandler buttonHandler;

    SplashScreen splashScreen;

    tap::Drivers *drivers;
};  // class OledDisplay
}  // namespace display
}  // namespace aruwsrc

#endif  // OLED_DISPLAY_HPP_
