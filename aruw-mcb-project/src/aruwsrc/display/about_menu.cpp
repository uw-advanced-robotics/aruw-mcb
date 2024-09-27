/*
 * Copyright (c) 2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "about_menu.hpp"

#include "tap/drivers.hpp"

#define STRINGIFYMACRO(s) MACROSTR(s)
#define MACROSTR(s) #s

namespace aruwsrc::display
{
#if defined(TARGET_STANDARD_ELSA)
static constexpr char ROBOT_NAME[] = "TARGET_STANDARD_ELSA";
#elif defined(TARGET_DRONE)
static constexpr char ROBOT_NAME[] = "TARGET_DRONE";
#elif defined(TARGET_ENGINEER)
static constexpr char ROBOT_NAME[] = "TARGET_ENGINEER";
#elif defined(TARGET_SENTRY_HYDRA)
static constexpr char ROBOT_NAME[] = "TARGET_SENTRY_HYDRA";
#elif defined(TARGET_HERO_CYCLONE)
static constexpr char ROBOT_NAME[] = "TARGET_HERO_PERSEUS";
#elif defined(TARGET_STANDARD_SPIDER)
static constexpr char ROBOT_NAME[] = "TARGET_STANDARD_SPIDER";
#elif defined(TARGET_STANDARD_ORION)
static constexpr char ROBOT_NAME[] = "TARGET_STANDARD_ORION";
#elif defined(TARGET_STANDARD_CYGNUS)
static constexpr char ROBOT_NAME[] = "TARGET_STANDARD_CYGNUS";
#else
static constexpr char ROBOT_NAME[] = "TARGET_UNKNOWN";
#endif

static constexpr char LAST_USER[] = STRINGIFYMACRO(BUILD_USERNAME);
static constexpr char LAST_SHA[] = STRINGIFYMACRO(BUILD_SHA);
static constexpr char LAST_DATE[] = STRINGIFYMACRO(BUILD_DATE);
static constexpr char BRANCH_NAME[] = STRINGIFYMACRO(BUILD_BRANCH_NAME);

AboutMenu::AboutMenu(modm::ViewStack<tap::display::DummyAllocator<modm::IAbstractView> > *vs)
    : AbstractMenu<tap::display::DummyAllocator<modm::IAbstractView> >(vs, TURRET_MCB_MENU_ID)

{
}

void AboutMenu::draw()
{
    modm::GraphicDisplay &display = getViewStack()->getDisplay();
    display.clear();
    display.setCursor(0, 2);
    display << getMenuName() << modm::endl;

    display << "Robot Name: " << ROBOT_NAME << modm::endl;
    display << "Last User: " << LAST_USER << modm::endl;
    display << "Sha: " << LAST_SHA << modm::endl;
    display << "Last Built: " << LAST_DATE << modm::endl;
    display << "Branch Name: " << BRANCH_NAME << modm::endl;
    drawn = true;
}

void AboutMenu::update() {}

void AboutMenu::shortButtonPress(modm::MenuButtons::Button button)
{
    if (button == modm::MenuButtons::LEFT)
    {
        this->remove();
    }
}
bool AboutMenu::hasChanged() { return !drawn; }

}  // namespace aruwsrc::display
