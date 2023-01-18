/*
 * Copyright (c) 2021-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef TAPROOT_STANDARD_FRAMES_HPP_
#define TAPROOT_STANDARD_FRAMES_HPP_

#include "tap/algorithms/transforms/frames.hpp"

namespace aruwsrc::algorithms
{

class WorldFrame : public tap::algorithms::Frame { } ;

class ChassisFrame : public tap::algorithms::Frame { } ;

class GunFrame : public tap::algorithms::Frame { } ;

// class TurretPivotFrame : public tap::algorithms::Frame { } ;

class CameraFrame : public tap::algorithms::Frame { } ;

class TurretIMUFrame : public tap::algorithms::Frame { } ;

class ChassisIMUFrame : public tap::algorithms::Frame { } ;

}

#endif // TAPROOT_STANDARD_FRAMES_HPP_
