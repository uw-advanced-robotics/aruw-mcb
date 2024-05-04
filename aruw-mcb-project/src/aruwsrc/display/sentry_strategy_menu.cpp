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

#include "sentry_strategy_menu.hpp"

#include "tap/drivers.hpp"

namespace aruwsrc::display
{
SentryStrategyMenu::SentryStrategyMenu(
    modm::ViewStack<tap::display::DummyAllocator<modm::IAbstractView>> *vs,
    aruwsrc::serial::VisionCoprocessor *visionCoprocessor)
    : modm::ChoiceMenu<tap::display::DummyAllocator<modm::IAbstractView>>(
        vs,
        SENTRY_STRATEGY_MENU_ID,
        getMenuName()),
        visionCoprocessor(visionCoprocessor)
{}

void SentryStrategyMenu::draw()
{
    modm::GraphicDisplay &display = getViewStack()->getDisplay();
    display.clear();
    display.setCursor(0, 2);
    display << getMenuName() << modm::endl;

    aruwsrc::serial::VisionCoprocessor::MotionStrategyOptionsData options = visionCoprocessor->getLastMotionStratOptionsData();
    std::string string_so_far;
    for (unsigned int i = 0; i < sizeof(options.motion_strategy_options); i++) {
        char c = options.motion_strategy_options[i];
        if (c == ',') {
            addEntry(
                string_so_far.c_str(),
                visionCoprocessor->getMutableMotionStrategyPtr(SentryVisionMessageType::NONE),
                false);
        } else {
            string_so_far += c;
        }
    }

    display.printf(options.motion_strategy_options);
}

    // addEntry(
    //     "None",
    //     visionCoprocessor->getMutableMotionStrategyPtr(SentryVisionMessageType::NONE),
    //     true);
    // addEntry(
    //     "Go crazy",
    //     visionCoprocessor->getMutableMotionStrategyPtr(SentryVisionMessageType::RUSH_BASE),
    //     false);
    // addEntry(
    //     "Go stupid",
    //     visionCoprocessor->getMutableMotionStrategyPtr(SentryVisionMessageType::GO_HEAL),
    //     false);
    // addEntry(
    //     "AAH",
    //     visionCoprocessor->getMutableMotionStrategyPtr(SentryVisionMessageType::RUSH_MID),
    //     false);

void SentryStrategyMenu::update() {}

void SentryStrategyMenu::shortButtonPress(modm::MenuButtons::Button button)
{
    if (button == modm::MenuButtons::LEFT)
    {
        this->remove();
    }
}

bool SentryStrategyMenu::hasChanged() { return updatePeriodicTimer.execute(); }

}  // namespace aruwsrc::display
