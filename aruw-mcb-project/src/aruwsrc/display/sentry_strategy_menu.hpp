/*
 * Copyright (c) 2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef SENTRY_STRATEGY_MENU_HPP_
#define SENTRY_STRATEGY_MENU_HPP_

#include <aruwsrc/communication/serial/sentry_strategy_message_types.hpp>

#include "tap/display/dummy_allocator.hpp"

#include "aruwsrc/communication/serial/vision_coprocessor.hpp"
#include "modm/ui/menu/choice_menu.hpp"

using namespace aruwsrc::serial;
// using namespace aruwsrc::communication::serial;

/**
 * Menu to setup menu options to go send to the sentry through vision coprocessor options
 */
namespace aruwsrc::display
{
class SentryStrategyMenu
    : public modm::ChoiceMenu<tap::display::DummyAllocator<modm::IAbstractView>>
{
public:
    SentryStrategyMenu(
        modm::ViewStack<tap::display::DummyAllocator<modm::IAbstractView>> *vs,
        VisionCoprocessor *visionCoprocessor)
        : modm::ChoiceMenu<tap::display::DummyAllocator<modm::IAbstractView>>(
              vs,
              SENTRY_STRATEGY_MENU_ID,
              getMenuName()),
          visionCoprocessor(visionCoprocessor)
    {
        addEntry(
            "None",
            visionCoprocessor->getMutableMotionStrategyPtr(
                aruwsrc::communication::serial::SentryVisionMessageType::NONE),
            true);
        addEntry(
            "Go crazy",
            visionCoprocessor->getMutableMotionStrategyPtr(
                aruwsrc::communication::serial::SentryVisionMessageType::RUSH_BASE),
            false);
        addEntry(
            "Go stupid",
            visionCoprocessor->getMutableMotionStrategyPtr(
                aruwsrc::communication::serial::SentryVisionMessageType::GO_HEAL),
            false);
        addEntry(
            "AAH",
            visionCoprocessor->getMutableMotionStrategyPtr(
                aruwsrc::communication::serial::SentryVisionMessageType::RUSH_MID),
            false);
    }

    void openNextScreen() override {}

    static const char *getMenuName() { return "Sentry Strategy Menu"; }

private:
    static constexpr int SENTRY_STRATEGY_MENU_ID = 13;

    VisionCoprocessor *visionCoprocessor;
};
}  // namespace aruwsrc::display

#endif  // SENTRY_STRATEGY_MENU_HPP_
