/*
 * Copyright (c) 2022-2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "sentry_response_subsystem.hpp"

#include "tap/drivers.hpp"
#include "tap/communication/serial/ref_serial_data.hpp"

namespace aruwsrc::communication::serial
{
SentryResponseSubsystem::SentryResponseSubsystem(tap::Drivers *drivers)
    : tap::control::Subsystem(drivers),
      sentryResponseTransmitter{*drivers, {tap::communication::serial::RefSerialData::RobotId::BLUE_HERO, tap::communication::serial::RefSerialData::RobotId::BLUE_SOLDIER_1}, SENTRY_RESPONSE_MESSAGE_ID}
{
}

void SentryResponseSubsystem::refresh() { sentryResponseTransmitter.sendQueued(); }

}  // namespace aruwsrc::communication::serial
