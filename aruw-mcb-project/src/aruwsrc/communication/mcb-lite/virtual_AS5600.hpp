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

#ifndef VIRTUAL_AS5600_HPP_
#define VIRTUAL_AS5600_HPP_

namespace aruwsrc::virtualMCB
{

class VirtualAS5600
{
    friend class MCBLite;

public:
    VirtualAS5600() {}

    float getPositionDegree() { return positionDegree; }

    // Degree per second
    float getVelocity() { return velocity; }

private:
    float positionDegree;
    float velocity;
};

}  // namespace aruwsrc::virtualMCB

#endif  // VIRTUAL_AS5600_HPP_
