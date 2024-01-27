/*
 * Copyright (c) 2020-2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "old_wheel.hpp"

namespace aruwsrc
{
class Drivers;
}

namespace aruwsrc
{
namespace algorithms
{
Wheel::Wheel(float diameter, float gearRatio, float motorGearRatio)
    : circumference(diameter * M_PI),
      gearRatio(gearRatio),
      motorGearRatio(motorGearRatio)
{
}

float Wheel::mpsToRpm(float mps) const
{
    return (mps / circumference) / motorGearRatio * 60.0f / gearRatio;
}

float Wheel::rpmToMps(float rpm) const
{
    return rpm * motorGearRatio / 60.0f * gearRatio * circumference;
}

}  // namespace algorithms

}  // namespace aruwsrc
