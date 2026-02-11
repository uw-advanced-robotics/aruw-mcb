/*
 * Copyright (c) 2026 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef CREATE_RTT_ERROR_HPP_
#define CREATE_RTT_ERROR_HPP_

#include "tap/drivers.hpp"
#include "tap/errors/system_error.hpp"

// Forward declaration
namespace aruwsrc::communication::rtt
{
class RttTelemetry;
}

namespace aruwsrc::communication::rtt
{
// Helper struct to access protected logError method
struct RttErrorHelper
{
    template <typename... Args>
    static void logError(RttTelemetry* telemetry, Args... args)
    {
        telemetry->logError(args...);
    }
};

}  // namespace aruwsrc::communication::rtt

namespace tap::errors
{
#undef RAISE_ERROR
#define RAISE_ERROR(drivers, telemetry, desc)                                     \
    do                                                                            \
    {                                                                             \
        tap::errors::SystemError stringError(desc, __LINE__, __FILE__);           \
        (drivers)->errorController.addToErrorList(stringError);                   \
        aruwsrc::communication::rtt::RttErrorHelper::logError((telemetry), desc); \
    } while (0)

}  // namespace tap::errors

#endif  // CREATE_RTT_ERROR_HPP_