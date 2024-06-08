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
{
    addEntry(
        "IDLE",
        visionCoprocessor->getMutableMotionStrategyPtr(SentryMotionStrategyType::IDLE),
        true);
    addEntry(
        "Default State Machine",
        visionCoprocessor->getMutableMotionStrategyPtr(
            SentryMotionStrategyType::DEFAULT_STATE_MACHINE),
        false);
    addEntry(
        "Practice Match State Machine",
        visionCoprocessor->getMutableMotionStrategyPtr(
            SentryMotionStrategyType::PRACTICE_MATCH_STATE_MACHINE),
        false);
    addEntry(
        "Test State Machine",
        visionCoprocessor->getMutableMotionStrategyPtr(
            SentryMotionStrategyType::TEST_STATE_MACHINE),
        false);
}

void SentryStrategyMenu::openNextScreen(){};
}  // namespace aruwsrc::display
