/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef ENGINEER_DRIVERS_HPP_
#define ENGINEER_DRIVERS_HPP_

#include "tap/drivers.hpp"

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
#include "aruwsrc/communication/serial/engineer_cv_communication.hpp"
#include "aruwsrc/mock/control_operator_interface_mock.hpp"
#include "aruwsrc/mock/oled_display_mock.hpp"

#else
#include "aruwsrc/communication/serial/engineer_cv_communication.hpp"
#include "aruwsrc/display/oled_display.hpp"
#include "aruwsrc/robot/control_operator_interface.hpp"
#include "aruwsrc/robot/engineer/engineer_control_operator_interface.hpp"
#endif

namespace aruwsrc::engineer
{
class Drivers : public tap::Drivers
{
    friend class DriversSingleton;

#ifdef ENV_UNIT_TESTS
public:
#endif

    Drivers()
        : tap::Drivers(),
          controlOperatorInterface(this),
          oledDisplay(this, nullptr, nullptr, nullptr, nullptr, nullptr),
          engineerCVCommunication(this)
    {
    }

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
    testing::NiceMock<mock::ControlOperatorInterfaceMock> controlOperatorInterface;
    testing::NiceMock<mock::OledDisplayMock> oledDisplay;
    serial::EngineerCVCommunication engineerCVCommunication;
#else
public:
    control::engineer::EngineerControlOperatorInterface controlOperatorInterface;
    display::OledDisplay oledDisplay;
    serial::EngineerCVCommunication engineerCVCommunication;
#endif
};  // class aruwsrc::EngineerDrivers
}  // namespace aruwsrc::engineer

#endif  // ENGINEER_DRIVERS_HPP_
