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

#include "rtt_menu.hpp"

namespace aruwsrc
{
namespace display
{
using namespace aruwsrc::communication::rtt;

RttMenu::RttMenu(
    modm::ViewStack<tap::display::DummyAllocator<modm::IAbstractView>> *vs,
    aruwsrc::communication::rtt::RttTelemetry *telemetry)
    : AbstractMenu<tap::display::DummyAllocator<modm::IAbstractView>>(vs, RTT_MENU_ID),
      telemetry(telemetry)
{
}

void RttMenu::draw()
{
    if (telemetry == nullptr) return;

    modm::GraphicDisplay &display = getViewStack()->getDisplay();
    display.clear();
    display.setCursor(0, 2);
    display << getMenuName() << modm::endl;

    display << "RTT Connection State: " << connectionStateToString(telemetry->getConnectionState())
            << modm::endl
            << "Message Queue Size: " << telemetry->getMessageQueueSize() << modm::endl
            << "Print Queue Size: " << telemetry->getPrintQueueSize() << modm::endl
            << "Error Queue Size: " << telemetry->getErrorQueueSize() << modm::endl
            << "Processing Log Msg: " << (telemetry->getProcessingLogMessage() ? "True" : "False")
            << modm::endl
            << "Processing Err Msg: " << (telemetry->getProcessingErrorMessage() ? "True" : "False")
            << modm::endl;
}

void RttMenu::update() {}

void RttMenu::shortButtonPress(modm::MenuButtons::Button button)
{
    if (button == modm::MenuButtons::LEFT)
    {
        this->remove();
    }
}

bool RttMenu::hasChanged() { return updatePeriodicTimer.execute(); }

}  // namespace display
}  // namespace aruwsrc
