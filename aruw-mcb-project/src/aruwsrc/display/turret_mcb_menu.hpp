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

#ifndef TURRET_MCB_MENU_HPP_
#define TURRET_MCB_MENU_HPP_

#include "tap/architecture/periodic_timer.hpp"
#include "tap/display/dummy_allocator.hpp"

#include "aruwsrc/communication/can/turret_mcb_can_comm.hpp"
#include "modm/ui/menu/abstract_menu.hpp"

namespace aruwsrc::display
{
/**
 * Menu that allows the user to schedule an `ImuCalibrateCommand` in the `CommandScheduler`. Also
 * displays the current calibration state of the `ImuCalibrationCommand`.
 */
class TurretMCBMenu : public modm::AbstractMenu<tap::display::DummyAllocator<modm::IAbstractView> >
{
public:
    /** Time between calls to `draw`, which will redraw the turret status menu. */
    static constexpr uint32_t DISPLAY_DRAW_PERIOD = 500;

    /**
     * @param[in] vs `ViewStack` that this menu is sitting on top of.
     * @param[in] drivers A pointer to the global drivers object.
     */
    TurretMCBMenu(
        modm::ViewStack<tap::display::DummyAllocator<modm::IAbstractView> > *vs,
        aruwsrc::can::TurretMCBCanComm *turretMCBCanComm);

    void draw() override;

    void update() override;

    void shortButtonPress(modm::MenuButtons::Button button) override;

    bool hasChanged() override;

    static const char *getMenuName() { return "Turret MCB Menu"; }

private:
    static constexpr int TURRET_MCB_MENU_ID = 12;

    aruwsrc::can::TurretMCBCanComm *turretMCBCanComm;

    tap::arch::PeriodicMilliTimer updatePeriodicTimer{DISPLAY_DRAW_PERIOD};
};
}  // namespace aruwsrc::display

#endif  // TURRET_MCB_MENU_HPP_
