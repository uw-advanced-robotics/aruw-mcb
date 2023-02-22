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

#ifndef ENV_UNIT_TESTS

#include "hero_drivers_singleton.hpp"

namespace aruwsrc
{
/**
 * Class that allows one to construct a Drivers instance because of friendship
 * with the Drivers class.
 */
class HeroDriversSingleton
{
public:
    static aruwsrc::HeroDrivers drivers;
};  // class DriversSingleton

aruwsrc::HeroDrivers HeroDriversSingleton::drivers;

aruwsrc::HeroDrivers *DoNotUse_getHeroDrivers() { return &HeroDriversSingleton::drivers; }
}  // namespace aruwsrc

#endif
