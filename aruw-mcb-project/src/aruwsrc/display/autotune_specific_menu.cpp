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

#include "autotune_specific_menu.hpp"

#include "tap/drivers.hpp"

#include "aruwsrc/control/autotune/gravity_autotune.hpp"

namespace aruwsrc::display
{
AutotuneSpecificMenu::AutotuneSpecificMenu(
    modm::ViewStack<tap::display::DummyAllocator<modm::IAbstractView>> *vs,
    tap::Drivers *drivers,
    aruwsrc::control::autotune::TurretAutotuneInterface *autotuneCommand)
    : AbstractMenu<tap::display::DummyAllocator<modm::IAbstractView>>(vs, AUTOTUNE_MENU_ID),
      drivers(drivers),
      autotuneCommand(autotuneCommand)
{
}

void AutotuneSpecificMenu::draw()
{
    modm::GraphicDisplay &display = getViewStack()->getDisplay();
    display.clear();
    display.setCursor(0, 2);
    display << getMenuName() << modm::endl;

    if (autotuneCommand == nullptr)
    {
        display << "No calibrate command";
    }
    else
    {
        display << CALI_STATE_TO_CHAR_STR[static_cast<int>(currCalibrationState)] << modm::endl;

        if (currCalibrationState == aruwsrc::control::autotune::TurretAutotuneInterface::
                                        CalibrationState::CALIBRATION_SUCCESS)
        {
            autotuneCommand->drawCalibrationResult(display);
        }
    }
}

void AutotuneSpecificMenu::update() {}

void AutotuneSpecificMenu::shortButtonPress(modm::MenuButtons::Button button)
{
    switch (button)
    {
        case modm::MenuButtons::LEFT:
            this->remove();
            if (autotuneCommand != nullptr &&
                drivers->commandScheduler.isCommandScheduled(autotuneCommand))
            {
                drivers->commandScheduler.removeCommand(autotuneCommand, true);
            }
            break;
        case modm::MenuButtons::OK:
            if (autotuneCommand != nullptr)
            {
                drivers->commandScheduler.addCommand(autotuneCommand);
            }
            break;
        case modm::MenuButtons::RIGHT:
        case modm::MenuButtons::DOWN:
        case modm::MenuButtons::UP:
        default:
            break;
    }
}

bool AutotuneSpecificMenu::hasChanged()
{
    using namespace aruwsrc::control::autotune;

    if (autotuneCommand == nullptr)
    {
        return false;
    }

    auto newCalibrationState = autotuneCommand->getCalibrationState();
    if (newCalibrationState != currCalibrationState)
    {
        currCalibrationState = newCalibrationState;
        return true;
    }
    return false;
}
}  // namespace aruwsrc::display
