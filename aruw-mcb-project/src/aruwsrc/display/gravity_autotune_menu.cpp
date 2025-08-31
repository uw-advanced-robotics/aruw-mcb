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

#include "gravity_autotune_menu.hpp"

#include "tap/drivers.hpp"

#include "aruwsrc/control/autotune/gravity_autotune.hpp"

namespace aruwsrc::display
{
GravityAutotuneMenu::GravityAutotuneMenu(
    modm::ViewStack<tap::display::DummyAllocator<modm::IAbstractView> > *vs,
    tap::Drivers *drivers,
    aruwsrc::control::autotune::GravityAutotuneBase *GravityAutotune)
    : AbstractMenu<tap::display::DummyAllocator<modm::IAbstractView> >(
          vs,
          GRAVITY_AUTOTUNE_MENU_ID),
      drivers(drivers),
      gravityAutotuneCommand(GravityAutotune)
{
}

void GravityAutotuneMenu::draw()
{
    modm::GraphicDisplay &display = getViewStack()->getDisplay();
    display.clear();
    display.setCursor(0, 2);
    display << getMenuName() << modm::endl;

    if (gravityAutotuneCommand == nullptr)
    {
        display << "No gravity calibrate command";
    }
    else
    {
        display << CALI_STATE_TO_CHAR_STR[static_cast<int>(currCalibrationState)] << modm::endl;

        if (currCalibrationState ==
            aruwsrc::control::autotune::GravityAutotuneBase::CalibrationState::CALIBRATION_SUCCESS)
        {
            const auto result = gravityAutotuneCommand->getCalibrationResult();
            const float X = result[0];
            const float Z = result[1];
            const float scalar = result[2];

            display.printf(
                "Center of mass position:\n\tcgX: %.2f mm\n\tcgZ: %.2f mm\n",
                static_cast<double>(X),
                static_cast<double>(Z));
            display.printf("Gravity Compensation\n Scalar: -%.1f\n", static_cast<double>(scalar));
        }
    }
}

void GravityAutotuneMenu::update() {}

void GravityAutotuneMenu::shortButtonPress(modm::MenuButtons::Button button)
{
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

bool GravityAutotuneMenu::hasChanged()
{
    using namespace aruwsrc::control::autotune;

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
