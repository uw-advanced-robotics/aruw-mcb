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

#ifndef SENTRY_REQUEST_SUBSYSTEM_HPP_
#define SENTRY_REQUEST_SUBSYSTEM_HPP_

#include "tap/control/subsystem.hpp"
#include "tap/util_macros.hpp"

#include "vision_coprocessor.hpp"

namespace aruwsrc
{
class Drivers;
}

namespace aruwsrc::communication::serial
{
class SentryArucoResetSubsystem : public tap::control::Subsystem
{
public:
    SentryArucoResetSubsystem(tap::Drivers *drivers, aruwsrc::serial::VisionCoprocessor &visionCoprocessor);

    void refresh() override;

private:
    aruwsrc::serial::VisionCoprocessor &sentryVisionCoprocessor;

};
}  // namespace aruwsrc::communication::serial

#endif  // SENTRY_REQUEST_SUBSYSTEM_HPP_
