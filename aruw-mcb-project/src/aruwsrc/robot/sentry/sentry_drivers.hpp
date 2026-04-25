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
#include "aruwsrc/communication/sensors/imu/ism330/ism330.hpp"
#include "aruwsrc/communication/serial/vision_coprocessor.hpp"
#include "aruwsrc/control/chassis/constants/chassis_constants.hpp"
#include "aruwsrc/display/oled_display.hpp"
#include "aruwsrc/robot/sentry/sentry_control_operator_interface.hpp"
#endif

namespace aruwsrc::sentry
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
              &chassisMcbLite,
              nullptr,
              &capacitorBank,
              &rttTelemetry),
          turretMCBCanCommBus1(this, tap::can::CanBus::CAN_BUS1),
          turretMCBCanCommBus2(this, tap::can::CanBus::CAN_BUS2),
          mpu6500TerminalSerialHandler(this, &this->mpu6500),
          capacitorBank(
              this,
              tap::can::CanBus::CAN_BUS1,
              aruwsrc::control::chassis::CAP_BANK_CAPACITANCE),
          chassisMcbLite(this, tap::communication::serial::Uart::Uart7),
          turretMajorImu(),
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
    aruwsrc::communication::sensors::imu::ism330::ISM330 turretMajorImu;
    aruwsrc::algorithms::PlateHitTracker plateHitTracker;
    aruwsrc::algorithms::strategy_state_machine::RMULStateMachine stateMachine;

    void init(const float mainLoopFrequency)
    {
        visionCoprocessor.initializeCV();
        turretMCBCanCommBus1.init();
        turretMCBCanCommBus2.init();
        oledDisplay.initialize();
        capacitorBank.initialize();
        mpu6500.setCalibrationSamples(4000);
        chassisMcbLite.initialize();
        modm::delay_ms(2000);
        turretMajorImu.initialize(mainLoopFrequency, 0.1f, 0.0f);
        turretMajorImu.setCalibrationSamples(4000);
    }

    void updateIo()
    {
        oledDisplay.updateDisplay();
        visionCoprocessor.updateSerial();
        chassisMcbLite.updateSerial();
        turretMajorImu.read();
        stateMachine.updateState();
    }

    void update()
    {
        plateHitTracker.update();
        turretMCBCanCommBus1.sendData();
        turretMCBCanCommBus2.sendData();
        oledDisplay.updateMenu();
        chassisMcbLite.sendData();
        turretMajorImu.periodicIMUUpdate();
        visionCoprocessor.sendMessage();
        rttTelemetry.updateTelemetryAsync();
        checkTurretMcbDisconnection(this);
    }

private:
    inline void checkTurretMcbDisconnection(Drivers* drivers)
    {
        bool turretMcbConnected = drivers->turretMCBCanCommBus1.isConnected() &&
                                  drivers->turretMCBCanCommBus2.isConnected();
        if (!turretMcbConnected &&
            drivers->mpu6500.getImuState() !=
                tap::communication::sensors::imu::ImuInterface::ImuState::IMU_CALIBRATING)
        {
            tap::buzzer::playNote(&drivers->pwm, 1000);
        }
        else
        {
            tap::buzzer::silenceBuzzer(&drivers->pwm);
        }
    }

#endif
};  // class aruwsrc::SentryDrivers
}  // namespace aruwsrc::sentry

#endif  // SENTRY_DRIVERS_HPP_
