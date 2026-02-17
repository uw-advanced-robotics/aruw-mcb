/*
 * Copyright (c) 2020-2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef SENTRY_DRIVERS_HPP_
#define SENTRY_DRIVERS_HPP_

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
#include "aruwsrc/algorithms/strategy_state_machine/rmul_state_machine.hpp"
#include "aruwsrc/communication/can/cap-bank/capacitor_bank.hpp"
#include "aruwsrc/communication/can/turret_mcb_can_comm.hpp"
#include "aruwsrc/communication/mcb-lite/mcb_lite.hpp"
#include "aruwsrc/communication/rtt/rtt_telemetry.hpp"
#include "aruwsrc/communication/sensors/imu/fused_imu.hpp"
#include "aruwsrc/communication/sensors/imu/ism330/ism330.hpp"
#include "aruwsrc/communication/serial/vision_coprocessor.hpp"
#include "aruwsrc/display/oled_display.hpp"
#include "aruwsrc/robot/sentry/sentry_control_operator_interface.hpp"
#endif

namespace aruwsrc::sentry
{
#if defined(TARGET_SENTRY_NAME)
using TurretMajorImuType = aruwsrc::communication::sensors::imu::FusedImu<2>;
#else
using TurretMajorImuType = aruwsrc::communication::sensors::imu::ism330::ISM330;
#endif

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
              &chassisMcbLite,
              nullptr,
              &capacitorBank),
          turretMCBCanCommBus1(this, tap::can::CanBus::CAN_BUS1),
          turretMCBCanCommBus2(this, tap::can::CanBus::CAN_BUS2),
          mpu6500TerminalSerialHandler(this, &this->mpu6500),
          capacitorBank(this, tap::can::CanBus::CAN_BUS1, CAP_BANK_CAPACITANCE),
          chassisMcbLite(this, tap::communication::serial::Uart::Uart7),
#if defined(TARGET_SENTRY_NAME)
          turretMajorPrimaryImu(
              aruwsrc::communication::sensors::imu::ism330::ISM330::ChipSelectPin::BOARD_SPI_NSS),
          turretMajorImuSecondary(
              aruwsrc::communication::sensors::imu::ism330::ISM330::ChipSelectPin::GPIO_D12_H_ROW),
          turretMajorImu(
              {&turretMajorPrimaryImu, &turretMajorImuSecondary},
              // Jetson is forward, X forward, Y left
              {tap::algorithms::transforms::Transform(-76.7f, -116.14f, 0.0f, 0.0f, 0.0f, 0.0f),
               tap::algorithms::transforms::Transform(-76.7f, 116.04f, 0.0f, 0.0f, 0.0f, M_PI)},
              {aruwsrc::communication::sensors::imu::FusedImu<2>::ImuType::ISM330DHCX,
               aruwsrc::communication::sensors::imu::FusedImu<2>::ImuType::ISM330DHCX}),
#else
          turretMajorImu(),
#endif
          plateHitTracker(this),
          stateMachine(refSerial, visionCoprocessor)
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
    aruwsrc::sentry::SentryControlOperatorInterface controlOperatorInterface;
    aruwsrc::communication::serial::VisionCoprocessor visionCoprocessor;
    display::OledDisplay oledDisplay;
    aruwsrc::communication::can::TurretMCBCanComm turretMCBCanCommBus1;
    aruwsrc::communication::can::TurretMCBCanComm turretMCBCanCommBus2;
    tap::communication::sensors::imu::ImuTerminalSerialHandler mpu6500TerminalSerialHandler;
    aruwsrc::communication::can::cap_bank::CapacitorBank capacitorBank;
    aruwsrc::communication::mcb_lite::MCBLite chassisMcbLite;
#if defined(TARGET_SENTRY_NAME)
    aruwsrc::communication::sensors::imu::ism330::ISM330 turretMajorPrimaryImu;
    aruwsrc::communication::sensors::imu::ism330::ISM330 turretMajorImuSecondary;
#endif
    TurretMajorImuType turretMajorImu;
    aruwsrc::algorithms::PlateHitTracker plateHitTracker;
    aruwsrc::algorithms::strategy_state_machine::RMULStateMachine stateMachine;
    static constexpr float CAP_BANK_CAPACITANCE = 4.358f;

#endif
};  // class aruwsrc::SentryDrivers
}  // namespace aruwsrc::sentry

#endif  // SENTRY_DRIVERS_HPP_
