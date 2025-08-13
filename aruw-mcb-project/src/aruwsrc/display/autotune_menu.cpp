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

#include "autotune_menu.hpp"

#include "tap/drivers.hpp"

#include "aruwsrc/control/autotune/gravity_autotune.hpp"

/**
 * Default function if getImuCalibrateCommand not defined by the user.
 */
modm_weak aruwsrc::control::autotune::GravityAutotuneBase *getGravityAutotuneCommand()
{
    return nullptr;
}

namespace aruwsrc::display
{
AutotuneMenu::AutotuneMenu(
    modm::ViewStack<tap::display::DummyAllocator<modm::IAbstractView> > *vs,
    tap::Drivers *drivers)
    : AbstractMenu<tap::display::DummyAllocator<modm::IAbstractView> >(vs, AUTOTUNE_MENU_ID),
      drivers(drivers)
{
}

void AutotuneMenu::draw()
{
    modm::GraphicDisplay &display = getViewStack()->getDisplay();
    display.clear();
    display.setCursor(0, 2);
    display << getMenuName() << modm::endl;

    if (getGravityAutotuneCommand() == nullptr)
    {
        display << "No gravity calibrate command";
    }
    else
    {
        display << CALI_STATE_TO_CHAR_STR[static_cast<int>(currCalibrationState)];
    }
}

void AutotuneMenu::update() {}

void AutotuneMenu::shortButtonPress(modm::MenuButtons::Button button)
{
    aruwsrc::control::autotune::GravityAutotuneBase *gravityAutotuneCommand =
        getGravityAutotuneCommand();

    switch (button)
    {
        case modm::MenuButtons::LEFT:
            this->remove();
            if (gravityAutotuneCommand != nullptr)
            {
                drivers->commandScheduler.removeCommand(gravityAutotuneCommand, true);
            }
            break;
        case modm::MenuButtons::OK:
            if (gravityAutotuneCommand != nullptr)
            {
                drivers->commandScheduler.addCommand(gravityAutotuneCommand);
            }
            break;
        case modm::MenuButtons::RIGHT:
        case modm::MenuButtons::DOWN:
        case modm::MenuButtons::UP:
        default:
            break;
    }
}

bool AutotuneMenu::hasChanged()
{
    using namespace aruwsrc::control::autotune;

    GravityAutotuneBase *gravityAutotuneCommand = getGravityAutotuneCommand();

    if (gravityAutotuneCommand == nullptr)
    {
        return false;
    }

    auto newCalibrationState = gravityAutotuneCommand->getCalibrationState();
    if (newCalibrationState != currCalibrationState)
    {
        currCalibrationState = newCalibrationState;
        return true;
    }
    return false;
}
}  // namespace aruwsrc::display
