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

#ifndef TRANSLATION_PROVIDER_INTERFACE_HPP_
#define TRANSLATION_PROVIDER_INTERFACE_HPP_

#include "tap/algorithms/transforms/position.hpp"

#include "frames.hpp"

using namespace tap::algorithms::transforms;

namespace aruwsrc::algorithms::state
{
template <Frame BASE, Frame FOLLOWER>
class TranslationProviderInterface
{
public:
    virtual Position getTranslation();
};

}  // namespace aruwsrc::algorithms::state

#endif  // TRANSLATION_PROVIDER_INTERFACE_HPP_
