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

#ifndef SPLASH_SCREEN_HPP_
#define SPLASH_SCREEN_HPP_

#include "tap/display/dummy_allocator.hpp"
#include "tap/drivers.hpp"

#include "aruwsrc/communication/can/capacitor_bank.hpp"
#include "aruwsrc/communication/mcb-lite/mcb_lite.hpp"
#include "aruwsrc/communication/serial/vision_coprocessor.hpp"
#include "modm/ui/menu/abstract_menu.hpp"

namespace aruwsrc
{
class Drivers;
}

namespace aruwsrc
{
namespace display
{
class SplashScreen : public modm::AbstractMenu<tap::display::DummyAllocator<modm::IAbstractView> >
{
public:
    SplashScreen(
        modm::ViewStack<tap::display::DummyAllocator<modm::IAbstractView> > *vs,
        tap::Drivers *drivers,
        serial::VisionCoprocessor *visionCoprocessor,
        can::TurretMCBCanComm *turretMCBCanCommBus1,
        can::TurretMCBCanComm *turretMCBCanCommBus2,
        aruwsrc::virtualMCB::MCBLite *mcbLite1,
        aruwsrc::virtualMCB::MCBLite *mcbLite2,
        can::capbank::CapacitorBank *capacitorBank);

    void draw() override;

    void update() override;

    void shortButtonPress(modm::MenuButtons::Button button) override;

    bool hasChanged() override;

    inline void resetHasChanged() { drawn = false; }

private:
    static constexpr int SPLASH_SCREEN_MENU_ID = 1;

    bool drawn = false;
    tap::Drivers *drivers;
    serial::VisionCoprocessor *visionCoprocessor;
    can::TurretMCBCanComm *turretMCBCanCommBus1;
    can::TurretMCBCanComm *turretMCBCanCommBus2;
    aruwsrc::virtualMCB::MCBLite *mcbLite1;
    aruwsrc::virtualMCB::MCBLite *mcbLite2;
    can::capbank::CapacitorBank *capacitorBank;
};
}  // namespace display
}  // namespace aruwsrc

#endif  // SPLASH_SCREEN_HPP_
