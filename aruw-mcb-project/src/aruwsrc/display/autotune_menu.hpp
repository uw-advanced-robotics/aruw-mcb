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

#ifndef IMU_CALIBRATE_MENU_HPP_
#define IMU_CALIBRATE_MENU_HPP_

#include "tap/display/dummy_allocator.hpp"

#include "aruwsrc/control/autotune/gravity_autotune.hpp"
#include "modm/ui/menu/abstract_menu.hpp"

namespace aruwsrc
{
class Drivers;
}  // namespace aruwsrc

/**
 * Weak function that you should define in `*_control.cpp` if an `ImuCalibrateCommand` exists.
 */
aruwsrc::control::autotune::GravityAutotuneBase *getGravityAutotuneCommand();

namespace aruwsrc::display
{
/**
 * Menu that allows the user to schedule an `ImuCalibrateCommand` in the `CommandScheduler`. Also
 * displays the current calibration state of the `ImuCalibrationCommand`.
 */
class AutotuneMenu : public modm::AbstractMenu<tap::display::DummyAllocator<modm::IAbstractView> >
{
public:
    /**
     * @param[in] vs `ViewStack` that this menu is sitting on top of.
     * @param[in] drivers A pointer to the global drivers object.
     */
    AutotuneMenu(
        modm::ViewStack<tap::display::DummyAllocator<modm::IAbstractView> > *vs,
        tap::Drivers *drivers);

    void draw() override;

    void update() override;

    void shortButtonPress(modm::MenuButtons::Button button) override;

    bool hasChanged() override;

    static const char *getMenuName() { return "Autotune Calibrate Menu"; }

private:
    static constexpr int AUTOTUNE_MENU_ID = 15;

    static constexpr const char *CALI_STATE_TO_CHAR_STR[] = {
        "WAIT SYSTEMS ONLINE",
        "LOCKING TURRET",
        "MEASURING_TORQUE",
        "NEXT_LOCATION"
        "CALIBRATION_SUCCESS",
        "CALIBRATION_FAIL",
        "DONE"};

    tap::Drivers *drivers;

    aruwsrc::control::autotune::GravityAutotuneBase::CalibrationState currCalibrationState =
        aruwsrc::control::autotune::GravityAutotuneBase::CalibrationState::
            WAITING_FOR_SYSTEMS_ONLINE;
};
}  // namespace aruwsrc::display

#endif  // IMU_CALIBRATE_MENU_HPP_
