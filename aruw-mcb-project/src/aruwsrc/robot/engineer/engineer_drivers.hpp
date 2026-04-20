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
#include "aruwsrc/mock/turret_mcb_can_comm_mock.hpp"

#else
#include "aruwsrc/communication/can/turret_mcb_can_comm.hpp"
#include "aruwsrc/communication/mcb-lite/mcb_lite.hpp"
#include "aruwsrc/communication/rtt/rtt_telemetry.hpp"
#include "aruwsrc/communication/sensors/imu/ism330/ism330.hpp"
#include "aruwsrc/communication/serial/engineer_cv_communication.hpp"
#include "aruwsrc/control/control_operator_interface.hpp"
#include "aruwsrc/display/oled_display.hpp"
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
          rttTelemetry(this),
          controlOperatorInterface(this),
          oledDisplay(
              this,
              nullptr,
              &turretMCBCanCommBus1,
              &turretMCBCanCommBus2,
              nullptr,
              nullptr,
              nullptr,
              &rttTelemetry),
          engineerCVCommunication(this),
          chassisIsm(),
          mcbLite(this, tap::communication::serial::Uart::Uart7),
          turretMCBCanCommBus1(this, tap::can::CanBus::CAN_BUS1),
          turretMCBCanCommBus2(this, tap::can::CanBus::CAN_BUS2)
    {
        controlOperatorInterface.setTelemetry(&rttTelemetry);
    }

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
    testing::NiceMock<mock::ControlOperatorInterfaceMock> controlOperatorInterface;
    testing::NiceMock<mock::OledDisplayMock> oledDisplay;
    serial::EngineerCVCommunication engineerCVCommunication;

    testing::NiceMock<mock::TurretMCBCanCommMock> turretMCBCanCommBus1;
    testing::NiceMock<mock::TurretMCBCanCommMock> turretMCBCanCommBus2;
#else
public:
    communication::rtt::RttTelemetry rttTelemetry;
    engineer::EngineerControlOperatorInterface controlOperatorInterface;
    display::OledDisplay oledDisplay;
    communication::serial::EngineerCVCommunication engineerCVCommunication;
    aruwsrc::communication::sensors::imu::ism330::ISM330 chassisIsm;
    aruwsrc::communication::mcb_lite::MCBLite mcbLite;
    communication::can::TurretMCBCanComm turretMCBCanCommBus1;
    communication::can::TurretMCBCanComm turretMCBCanCommBus2;

    void init(const float mainLoopFrequency)
    {
        turretMCBCanCommBus1.init();
        turretMCBCanCommBus2.init();
        engineerCVCommunication.initializeCV();
        oledDisplay.initialize();
        digital.configureInputPullMode(
            tap::gpio::Digital::B,
            tap::gpio::Digital::InputPullMode::PullUp);
        digital.configureInputPullMode(
            tap::gpio::Digital::D,
            tap::gpio::Digital::InputPullMode::PullUp);
        digital.configureInputPullMode(
            tap::gpio::Digital::T,
            tap::gpio::Digital::InputPullMode::PullUp);
        chassisIsm.initialize(mainLoopFrequency, 0.1f, 0.0f);
        chassisIsm.setCalibrationSamples(4000);
        mcbLite.initialize();

        mcbLite.imu.initialize(mainLoopFrequency, 0.2f, 0.0f);
        // mcbLite.imu.requestCalibration();
    }

    void updateIo()
    {
        oledDisplay.updateDisplay();
        engineerCVCommunication.updateSerial();
        chassisIsm.read();
        mcbLite.updateSerial();
    }

    void update()
    {
        turretMCBCanCommBus1.sendData();
        turretMCBCanCommBus2.sendData();
        mcbLite.sendData();
        oledDisplay.updateMenu();
        rttTelemetry.updateTelemetryAsync();
        chassisIsm.periodicIMUUpdate();
    }
#endif
};  // class aruwsrc::EngineerDrivers
}  // namespace aruwsrc::engineer

#endif  // ENGINEER_DRIVERS_HPP_
