/*
 * Copyright (c) 2023-2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef MOTOR_TESTER_DRIVERS_HPP_
#define MOTOR_TESTER_DRIVERS_HPP_

#include "tap/drivers.hpp"

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
#include "aruwsrc/mock/rtt_telemetry_mock.hpp"
#else
#include "aruwsrc/communication/rtt/rtt_telemetry.hpp"
#endif
#include "aruwsrc/display/oled_display.hpp"

namespace aruwsrc::motor_tester
{
class Drivers : public tap::Drivers
{
    friend class DriversSingleton;

#ifdef ENV_UNIT_TESTS
public:
#endif
    Drivers()
        : tap::Drivers(),
          rttTelemetry(this),
          oledDisplay(this, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr)
    {
    }

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
    testing::NiceMock<mock::RttTelemetryMock> rttTelemetry;
#else
public:
    communication::rtt::RttTelemetry rttTelemetry;
#endif
    display::OledDisplay oledDisplay;
};  // class aruwsrc::MotortesterDrivers
}  // namespace aruwsrc::motor_tester

#endif  // MOTOR_TESTER_DRIVERS_HPP_
