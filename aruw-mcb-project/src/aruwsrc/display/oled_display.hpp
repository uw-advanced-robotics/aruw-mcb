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

#include "tap/architecture/periodic_timer.hpp"
#include "tap/board/board.hpp"
#include "tap/display/oled_button_handler.hpp"
#include "tap/display/sh1106/sh1106.hpp"
#include "tap/display/sh1107/sh1107.hpp"
#include "tap/util_macros.hpp"

#include "aruwsrc/communication/mcb-lite/mcb_lite.hpp"
#include "modm/processing/protothread.hpp"
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
class OledDisplay : public ::modm::pt::Protothread
{
public:
    explicit OledDisplay(
        tap::Drivers *drivers,
        communication::serial::VisionCoprocessor *visionCoprocessor,
        communication::can::TurretMCBCanComm *turretMCBCanCommBus1,
        communication::can::TurretMCBCanComm *turretMCBCanCommBus2,
        aruwsrc::communication::mcb_lite::MCBLite *mcbLite1,
        aruwsrc::communication::mcb_lite::MCBLite *mcbLite2,
        communication::can::cap_bank::CapacitorBank *capacitorBank,
        aruwsrc::communication::rtt::RttTelemetry *rttTelemetry);
    DISALLOW_COPY_AND_ASSIGN(OledDisplay)
    mockable ~OledDisplay() = default;

    mockable void initialize();

    /**
     * Updates the display in a nonblocking fashion. This function uses protothreads
     * to call the display's updateNonblocking function at a rate of 2 hz.
     *
     * @note This function uses protothreads (http://dunkels.com/adam/pt/).
     *      Local variables *do not* necessarily behave correctly and this
     *      function should be edited with care.
     */
    mockable bool updateDisplay();

    /**
     * Checks button state and updates the view stack responsible for determining what
     * should be displayed on the OLED.
     */
    mockable void updateMenu();

private:
    tap::display::OledButtonHandler::Button prevButton = tap::display::OledButtonHandler::NONE;

#ifdef SSH1106_OLED
    tap::display::Sh1106<
#ifndef PLATFORM_HOSTED
        Board::DisplaySpi,
        Board::DisplayCommand,
        Board::DisplayReset,
#endif
        128,
        64,
        false>
        display;
#else
    tap::display::Sh1107<
#ifndef PLATFORM_HOSTED
        Board::DisplaySpi,
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

#ifndef SSH1106_OLED
    const tap::display::AnalogConfig buttonConfig =
        {.ok = 50, .left = 1000, .right = 2000, .up = 3050, .down = 3700};
#endif

    tap::display::OledButtonHandler buttonHandler;

    SplashScreen splashScreen;

    tap::Drivers *drivers;

    tap::arch::PeriodicMilliTimer displayThreadTimer{100};
};  // class OledDisplay

}  // namespace display
}  // namespace aruwsrc

#endif  // OLED_DISPLAY_HPP_
