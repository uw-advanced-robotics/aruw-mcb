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

#ifndef DRONE_DRIVERS_HPP_
#define DRONE_DRIVERS_HPP_

#include "tap/drivers.hpp"

#include "aruwsrc/control/turret/constants/turret_constants.hpp"

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
#include "tap/mock/imu_terminal_serial_handler_mock.hpp"

#include "aruwsrc/mock/control_operator_interface_mock.hpp"
#include "aruwsrc/mock/oled_display_mock.hpp"
#include "aruwsrc/mock/vision_coprocessor_mock.hpp"
#else
#include "tap/communication/sensors/imu/imu_terminal_serial_handler.hpp"

#include "aruwsrc/communication/rtt/rtt_telemetry.hpp"
#include "aruwsrc/communication/serial/vision_coprocessor.hpp"
#include "aruwsrc/display/oled_display.hpp"
#include "aruwsrc/robot/drone/drone_control_operator_interface.hpp"
#include "aruwsrc/robot/drone/drone_imu.hpp"
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
          visionCoprocessor(this),
          rttTelemetry(this),
          turretImu(),
          oledDisplay(
              this,
              &visionCoprocessor,
              nullptr,
              nullptr,
              nullptr,
              nullptr,
              nullptr,
              &rttTelemetry)
#if !defined(PLATFORM_HOSTED) || !defined(ENV_UNIT_TESTS)
    {
        controlOperatorInterface.setTelemetry(&rttTelemetry);
        visionCoprocessor.setTelemetry(&rttTelemetry);
    }
#else
    {
    }
#endif

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
    testing::NiceMock<mock::ControlOperatorInterfaceMock> controlOperatorInterface;
    testing::NiceMock<mock::VisionCoprocessorMock> visionCoprocessor;
    testing::NiceMock<mock::OledDisplayMock> oledDisplay;
#else
public:
    DroneControlOperatorInterface controlOperatorInterface;
    communication::serial::VisionCoprocessor visionCoprocessor;
#endif
    communication::rtt::RttTelemetry rttTelemetry;
    DroneIMU turretImu;
    display::OledDisplay oledDisplay;
    void init(const float mainLoopFrequency)
    {
#if !defined(PLATFORM_HOSTED) || !defined(ENV_UNIT_TESTS)
        visionCoprocessor.initializeCV();
#endif
        oledDisplay.initialize();
        turretImu.initialize(mainLoopFrequency, 0.1f, 0.0f);
        turretImu.setMountingTransform(
            aruwsrc::control::turret::TURRET_IMU_CALIBRATION_MOUNTING_TRANSFORM);
        turretImu.setCalibrationSamples(4000);
    }

    void updateIo()
    {
        oledDisplay.updateDisplay();
        turretImu.read();
#if !defined(PLATFORM_HOSTED) || !defined(ENV_UNIT_TESTS)
        visionCoprocessor.updateSerial();
#endif
    }

    void update()
    {
        turretImu.periodicIMUUpdate();
        oledDisplay.updateMenu();
#if !defined(PLATFORM_HOSTED) || !defined(ENV_UNIT_TESTS)
        visionCoprocessor.sendMessage();
#endif
        rttTelemetry.updateTelemetryAsync();
    }
};  // class aruwsrc::DroneDrivers
}  // namespace aruwsrc::drone

#endif  // DRONE_DRIVERS_HPP_
