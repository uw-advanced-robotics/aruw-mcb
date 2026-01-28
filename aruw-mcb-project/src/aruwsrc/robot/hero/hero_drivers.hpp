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

#ifndef HERO_DRIVERS_HPP_
#define HERO_DRIVERS_HPP_

#include "tap/drivers.hpp"

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
#include "tap/mock/imu_terminal_serial_handler_mock.hpp"

#include "aruwsrc/mock/control_operator_interface_mock.hpp"
#include "aruwsrc/mock/oled_display_mock.hpp"
#include "aruwsrc/mock/turret_mcb_can_comm_mock.hpp"
#include "aruwsrc/mock/vision_coprocessor_mock.hpp"
#else
#include "tap/communication/sensors/imu/imu_terminal_serial_handler.hpp"

#include "aruwsrc/algorithms/plate_hit_tracker.hpp"
#include "aruwsrc/communication/can/turret_mcb_can_comm.hpp"
#include "aruwsrc/communication/inter_robot_comm/inter_robot_transmitter.hpp"
#include "aruwsrc/communication/rtt/rtt_telemetry.hpp"
#include "aruwsrc/communication/sensors/imu/ism330/ism330.hpp"
#include "aruwsrc/communication/serial/vision_coprocessor.hpp"
#include "aruwsrc/control/control_operator_interface.hpp"
#include "aruwsrc/display/oled_display.hpp"

#endif

namespace aruwsrc::hero
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
          controlOperatorInterface(this),
          visionCoprocessor(this),
          oledDisplay(
              this,
              &visionCoprocessor,
              &turretMCBCanCommBus1,
              &turretMCBCanCommBus2,
              nullptr,
              nullptr,
              &capacitorBank),
          turretMCBCanCommBus1(this, tap::can::CanBus::CAN_BUS1),
          turretMCBCanCommBus2(this, tap::can::CanBus::CAN_BUS2),
          mpu6500TerminalSerialHandler(this, &this->mpu6500),
          capacitorBank(this, tap::can::CanBus::CAN_BUS1, 4.358),
          plateHitTracker(this),
          refSerialTransmitter(this),
          interRobotTransmitter(&this->refSerial, &refSerialTransmitter, &this->visionCoprocessor)
    {
        controlOperatorInterface.setTelemetry(&rttTelemetry);
        visionCoprocessor.setTelemetry(&rttTelemetry);
    }

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
    testing::NiceMock<mock::ControlOperatorInterfaceMock> controlOperatorInterface;
    testing::NiceMock<mock::VisionCoprocessorMock> visionCoprocessor;
    testing::NiceMock<mock::OledDisplayMock> oledDisplay;
    testing::NiceMock<mock::TurretMCBCanCommMock> turretMCBCanCommBus1;
    testing::NiceMock<mock::TurretMCBCanCommMock> turretMCBCanCommBus2;
    testing::NiceMock<tap::mock::ImuTerminalSerialHandlerMock> mpu6500TerminalSerialHandler;
#else
public:
    communication::rtt::RttTelemetry rttTelemetry;
    control::ControlOperatorInterface controlOperatorInterface;
    communication::serial::VisionCoprocessor visionCoprocessor;
    display::OledDisplay oledDisplay;
    communication::can::TurretMCBCanComm turretMCBCanCommBus1;
    communication::can::TurretMCBCanComm turretMCBCanCommBus2;
    tap::communication::sensors::imu::ImuTerminalSerialHandler mpu6500TerminalSerialHandler;
    communication::can::cap_bank::CapacitorBank capacitorBank;
    algorithms::PlateHitTracker plateHitTracker;
    RefSerialTransmitter refSerialTransmitter;
    aruwsrc::communication::inter_robot_comm::InterRobotTransmitter interRobotTransmitter;
    // aruwsrc::communication::sensors::imu::ism330::ISM330<Board::I2CMaster> ism330;
#endif
};  // class aruwsrc::HeroDrivers
}  // namespace aruwsrc::hero

#endif  // HERO_DRIVERS_HPP_
