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
#ifndef SENTRY_TRANSFORM_SUBSYSTEM_HPP_
#define SENTRY_TRANSFORM_SUBSYSTEM_HPP_

#include "tap/control/subsystem.hpp"

#include "aruwsrc/robot/sentry/sentry_transforms.hpp"

namespace aruwsrc::sentry
{
class SentryTransformSubystem : public tap::control::Subsystem
{
public:
    SentryTransformSubystem(tap::Drivers& drivers, aruwsrc::sentry::SentryTransforms& transformer)
        : tap::control::Subsystem(&drivers),
          transformer(transformer)
    {
    }

    inline void initialize() override { transformer.initialize(); };
    inline void refresh() override { transformer.updateTransforms(); };

private:
    aruwsrc::sentry::SentryTransforms& transformer;
};

}  // namespace aruwsrc::sentry

#endif  // SENTRY_TRANSFORM_SUBSYSTEM_HPP_
