/*
 * Copyright (c) 2025-2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef TRANSFORM_PROVIDER_INTERFACE_HPP_
#define TRANSFORM_PROVIDER_INTERFACE_HPP_

#include "tap/algorithms/transforms/transform.hpp"

#include "frames.hpp"
#include "orientation_provider_interface.hpp"
#include "translation_provider_interface.hpp"

using namespace tap::algorithms::transforms;

namespace aruwsrc::algorithms::state
{
template <Frame BASE, Frame FOLLOWER>
class TransformProviderInterface : public TranslationProviderInterface<BASE, FOLLOWER>,
                                   public OrientationProviderInterface<BASE, FOLLOWER>
{
public:
    inline Transform getTransform() { return Transform(getTranslation(), getOrientation()) }
};

}  // namespace aruwsrc::algorithms::state

#endif  // TRANSFORM_PROVIDER_INTERFACE_HPP_
