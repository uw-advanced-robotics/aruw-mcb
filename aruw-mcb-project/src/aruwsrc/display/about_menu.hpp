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

#ifndef ABOUT_MENU_HPP_
#define ABOUT_MENU_HPP_

#define STRINGIFYMACRO(s) MACROSTR(s)
#define MACROSTR(s) #s

#include "tap/architecture/periodic_timer.hpp"
#include "tap/display/dummy_allocator.hpp"

#include "modm/ui/menu/abstract_menu.hpp"

namespace aruwsrc
{
class Drivers;
}  // namespace aruwsrc

namespace aruwsrc::display
{
/**
 * Menu that allows user to see information about the robot, who deployed the last code,
 * and the sha.
 */
class AboutMenu : public modm::AbstractMenu<tap::display::DummyAllocator<modm::IAbstractView> >
{
public:
#if defined(TARGET_STANDARD_ELSA)
    static constexpr char ROBOT_NAME[] = "TARGET_STANDARD_ELSA";
#elif defined(TARGET_DRONE)
    static constexpr char ROBOT_NAME[] = "TARGET_DRONE";
#elif defined(TARGET_ENGINEER)
    static constexpr char ROBOT_NAME[] = "TARGET_ENGINEER";
#elif defined(TARGET_SENTRY_HYDRA)
    static constexpr char ROBOT_NAME[] = "TARGET_SENTRY_HYDRA";
#elif defined(TARGET_HERO_CYCLONE)
    static constexpr char ROBOT_NAME[] = "TARGET_HERO_CYCLONE";
#else
    static constexpr char ROBOT_NAME[] = "UNKNOWN";
#endif

    static constexpr char LAST_USER[] = STRINGIFYMACRO(BUILD_USERNAME);
    static constexpr char LAST_SHA[] = STRINGIFYMACRO(BUILD_SHA);
    static constexpr char LAST_DATE[] = STRINGIFYMACRO(BUILD_DATE);
    static constexpr char BRANCH_NAME[] = STRINGIFYMACRO(BUILD_BRANCH_NAME);

    AboutMenu(modm::ViewStack<tap::display::DummyAllocator<modm::IAbstractView> > *vs);
    void draw() override;

    void update() override;

    void shortButtonPress(modm::MenuButtons::Button button) override;

    bool hasChanged() override;

    static const char *getMenuName() { return "About Menu"; }

    inline void resetHasChanged() { drawn = false; }

private:
    static constexpr int TURRET_MCB_MENU_ID = 7;

    bool drawn = false;
};
}  // namespace aruwsrc::display

#endif  // ABOUT_MENU_HPP_
