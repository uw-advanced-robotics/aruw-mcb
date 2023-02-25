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

#ifndef DART_DRIVERS_HPP_
#define DART_DRIVERS_HPP_

#include "tap/drivers.hpp"

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
#include "aruwsrc/mock/control_operator_interface_mock.hpp"
#include "aruwsrc/mock/turret_mcb_can_comm_mock.hpp"
#else

#include "aruwsrc/communication/can/turret_mcb_can_comm.hpp"
#include "aruwsrc/robot/control_operator_interface.hpp"
#endif

namespace aruwsrc
{
class DartDrivers : public tap::Drivers
{
    friend class DartDriversSingleton;

#ifdef ENV_UNIT_TESTS
public:
#endif
    DartDrivers()
        : tap::Drivers(),
          controlOperatorInterface(this),
          turretMCBCanCommBus1(this, tap::can::CanBus::CAN_BUS1)
    {
    }

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
    testing::NiceMock<mock::ControlOperatorInterfaceMock> controlOperatorInterface;
    testing::NiceMock<mock::TurretMCBCanCommMock> turretMCBCanCommBus1;
#else
public:
    control::ControlOperatorInterface controlOperatorInterface;
    can::TurretMCBCanComm turretMCBCanCommBus1;
#endif
};  // class aruwsrc::DartDrivers
}  // namespace aruwsrc

#endif  // DART_DRIVERS_HPP_
