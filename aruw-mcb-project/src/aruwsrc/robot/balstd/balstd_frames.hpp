/*
 * Copyright (c) 2021-2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef BALSTD_FRAME_HPP_
#define BALSTD_FRAME_HPP_

#include "tap/algorithms/transforms/frame.hpp"

namespace aruwsrc::balstd::transforms
{
/**
 * Frame is an empty class to provide type-checking for
 * generic Transforms. This class is intended to be inherited
 * by more specific frame subclasses, which should also be empty.
 */

class ChassisFrame : tap::algorithms::transforms::Frame
{
};
}  // namespace tap::algorithms::transforms

#endif  // TAPROOT_FRAME_HPP_