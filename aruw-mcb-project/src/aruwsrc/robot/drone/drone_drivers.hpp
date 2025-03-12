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
/*
#ifndef DRONE_DRIVERS_HPP_
#define DRONE_DRIVERS_HPP_

#include "tap/drivers.hpp"

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)

#else

#endif

namespace aruwsrc::drone
{
class Drivers : public tap::Drivers
{
    friend class DriversSingleton;

#ifdef ENV_UNIT_TESTS
public:
#endif
    Drivers() : tap::Drivers() {}

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)

#else
public:

#endif
};  // class aruwsrc::DroneDrivers
}  // namespace aruwsrc::drone

#endif  // DRONE_DRIVERS_HPP_
*/

#ifndef DRONE_DRIVERS_HPP_
#define DRONE_DRIVERS_HPP_

#include "tap/drivers.hpp"

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
#include "tap/mock/imu_terminal_serial_handler_mock.hpp"

#include "aruwsrc/mock/control_operator_interface_mock.hpp"
#include "aruwsrc/mock/turret_mcb_can_comm_mock.hpp"
#else
#include "tap/communication/sensors/imu/imu_terminal_serial_handler.hpp"

#include "aruwsrc/communication/can/turret_mcb_can_comm.hpp"
#include "aruwsrc/robot/control_operator_interface.hpp"
#include "aruwsrc/communication/serial/vision_coprocessor.hpp"
#endif

namespace aruwsrc::drone
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
          turretMCBCanCommBus1(this, tap::can::CanBus::CAN_BUS1),
          turretMCBCanCommBus2(this, tap::can::CanBus::CAN_BUS2)
    {
    }

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
    testing::NiceMock<mock::ControlOperatorInterfaceMock> controlOperatorInterface;
    testing::NiceMock<mock::TurretMCBCanCommMock> turretMCBCanCommBus1;
    testing::NiceMock<mock::TurretMCBCanCommMock> turretMCBCanCommBus2;
#else
public:
    control::ControlOperatorInterface controlOperatorInterface;
    can::TurretMCBCanComm turretMCBCanCommBus1;
    can::TurretMCBCanComm turretMCBCanCommBus2;
#endif
};

}  // namespace aruwsrc::drone

#endif  // DRONE_DRIVERS_HPP_