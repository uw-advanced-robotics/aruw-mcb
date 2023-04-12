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

#ifndef SENTRY_REQUEST_SUBSYSTEM_MOCK_HPP_
#define SENTRY_REQUEST_SUBSYSTEM_MOCK_HPP_

#include <gmock/gmock.h>

#include "aruwsrc/communication/serial/sentry_request_subsystem.hpp"

namespace aruwsrc::mock
{
class SentryRequestSubsystemMock : public communication::serial::SentryRequestSubsystem
{
public:
    SentryRequestSubsystemMock(tap::Drivers *drivers);
    ~SentryRequestSubsystemMock();

    MOCK_METHOD(void, queueRequest, (communication::serial::SentryRequestMessageType), (override));
};
}  // namespace aruwsrc::mock

#endif  // SENTRY_REQUEST_SUBSYSTEM_MOCK_HPP_
