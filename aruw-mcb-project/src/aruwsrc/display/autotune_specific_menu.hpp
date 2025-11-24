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

#ifndef GRAVITY_AUTOTUNE_MENU_HPP_
#define GRAVITY_AUTOTUNE_MENU_HPP_

#include <modm/io/iostream.hpp>

#include "tap/display/dummy_allocator.hpp"

#include "aruwsrc/control/autotune/gravity_autotune.hpp"
#include "modm/ui/menu/abstract_menu.hpp"

namespace aruwsrc
{
class Drivers;
}  // namespace aruwsrc

namespace aruwsrc::display
{
/**
 * Menu that allows the user to schedule an `autotuneCommand` in the `CommandScheduler`. Also
 * displays the current calibration state of the `autotuneCommand`.
 */
class AutotuneSpecificMenu
    : public modm::AbstractMenu<tap::display::DummyAllocator<modm::IAbstractView>>
{
public:
    /**
     * @param[in] vs `ViewStack` that this menu is sitting on top of.
     * @param[in] drivers A pointer to the global drivers object.
     */
    AutotuneSpecificMenu(
        modm::ViewStack<tap::display::DummyAllocator<modm::IAbstractView>> *vs,
        tap::Drivers *drivers,
        aruwsrc::control::autotune::TurretAutotuneInterface *autotuneCommand);

    void draw() override;

    void update() override;

    void shortButtonPress(modm::MenuButtons::Button button) override;

    bool hasChanged() override;

    static const char *getMenuName() { return "Run Calibrate Menu"; }

private:
    static constexpr int AUTOTUNE_MENU_ID = 1;

    static constexpr const char *CALI_STATE_TO_CHAR_STR[] = {
        "WAITING_FOR_SYSTEMS_ONLINE",
        "LOCKING_TURRET",
        "MEASURING_TORQUE",
        "NEXT_LOCATION",
        "CALIBRATION_SUCCESS",
        "CALIBRATION_FAIL",
        "DONE"};

    tap::Drivers *drivers;

    aruwsrc::control::autotune::TurretAutotuneInterface *autotuneCommand;

    aruwsrc::control::autotune::TurretAutotuneInterface::CalibrationState currCalibrationState =
        aruwsrc::control::autotune::TurretAutotuneInterface::CalibrationState::
            WAITING_FOR_SYSTEMS_ONLINE;
};
}  // namespace aruwsrc::display

#endif  // GRAVITY_AUTOTUNE_MENU_HPP_
