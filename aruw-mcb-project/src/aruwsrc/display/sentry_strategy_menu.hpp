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

#include "tap/display/dummy_allocator.hpp"

#include "aruwsrc/communication/serial/vision_coprocessor.hpp"
#include "modm/ui/menu/choice_menu.hpp"

using namespace aruwsrc::serial;
using namespace aruwsrc::communication::serial;


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
              SENTRY_STRATEGY_MENU_ID),
          visionCoprocessor(visionCoprocessor)
    {
        this->setTitle("Sentry Strategy Menu");
        addEntry("None", visionCoprocessor->writeToMotionStrategy(SentryVisionMessageType::NONE));
        addEntry("Go crazy", visionCoprocessor->writeToMotionStrategy(SentryVisionMessageType::GO_CRAZY));
        addEntry("Go stupid", visionCoprocessor->writeToMotionStrategy(SentryVisionMessageType::GO_STUPID));
        addEntry("AAH", visionCoprocessor->writeToMotionStrategy(SentryVisionMessageType::AHHHHH));
    }

    void openNextScreen() override {}

private:
    static constexpr int SENTRY_STRATEGY_MENU_ID = 13;

    VisionCoprocessor *visionCoprocessor;
};
}  // namespace aruwsrc::display

#endif  // SENTRY_STRATEGY_MENU_HPP_
