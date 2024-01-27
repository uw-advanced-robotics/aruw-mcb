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
#ifndef OLD_WHEEL_HPP_
#define OLD_WHEEL_HPP_

#include "modm/math/geometry/angle.hpp"

namespace aruwsrc
{
class Drivers;
}

namespace aruwsrc
{
namespace algorithms
{
/**
 *
 * A simple wheel class to handle gear ratio and encoder calculations
 *
 */
class Wheel
{
public:
    /**
     * @param diameter in meters
     */
    Wheel(float diameter, float gearRatio, float motorGearRatio);

    float mpsToRpm(float mps) const;
    float rpmToMps(float rpm) const;

private:
    const float circumference, gearRatio, motorGearRatio;

};  // class Wheel

}  // namespace algorithms

}  // namespace aruwsrc

#endif  // WHEEL_HPP_
