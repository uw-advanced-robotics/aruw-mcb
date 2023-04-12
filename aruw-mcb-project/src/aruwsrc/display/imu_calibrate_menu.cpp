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

#include "imu_calibrate_menu.hpp"

#include "tap/drivers.hpp"

#include "aruwsrc/control/imu/imu_calibrate_command.hpp"

/**
 * Default function if getImuCalibrateCommand not defined by the user.
 */
modm_weak aruwsrc::control::imu::ImuCalibrateCommand *getImuCalibrateCommand() { return nullptr; }

namespace aruwsrc::display
{
ImuCalibrateMenu::ImuCalibrateMenu(
    modm::ViewStack<tap::display::DummyAllocator<modm::IAbstractView> > *vs,
    tap::Drivers *drivers)
    : AbstractMenu<tap::display::DummyAllocator<modm::IAbstractView> >(vs, IMU_CALIBRATE_MENU_ID),
      drivers(drivers)
{
}

void ImuCalibrateMenu::draw()
{
    modm::GraphicDisplay &display = getViewStack()->getDisplay();
    display.clear();
    display.setCursor(0, 2);
    display << getMenuName() << modm::endl;

    if (getImuCalibrateCommand() == nullptr)
    {
        display << "No IMU calibrate command";
    }
    else
    {
        display << CALI_STATE_TO_CHAR_STR[static_cast<int>(currCalibrationState)];
    }
}

void ImuCalibrateMenu::update() {}

void ImuCalibrateMenu::shortButtonPress(modm::MenuButtons::Button button)
{
    control::imu::ImuCalibrateCommand *imuCalibrateCommand = getImuCalibrateCommand();

    switch (button)
    {
        case modm::MenuButtons::LEFT:
            this->remove();
            if (imuCalibrateCommand != nullptr)
            {
                drivers->commandScheduler.removeCommand(imuCalibrateCommand, true);
            }
            break;
        case modm::MenuButtons::OK:
            if (imuCalibrateCommand != nullptr)
            {
                drivers->commandScheduler.addCommand(imuCalibrateCommand);
            }
            break;
        case modm::MenuButtons::RIGHT:
        case modm::MenuButtons::DOWN:
        case modm::MenuButtons::UP:
        default:
            break;
    }
}

bool ImuCalibrateMenu::hasChanged()
{
    control::imu::ImuCalibrateCommand *imuCalibrateCommand = getImuCalibrateCommand();

    if (imuCalibrateCommand == nullptr)
    {
        return false;
    }

    auto newCalibrationState = imuCalibrateCommand->getCalibrationState();
    if (newCalibrationState != currCalibrationState)
    {
        currCalibrationState = newCalibrationState;
        return true;
    }
    return false;
}
}  // namespace aruwsrc::display
