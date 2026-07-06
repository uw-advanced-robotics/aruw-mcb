/*
 * Copyright (c) 2025-2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef BALSTD_DRIVERS_HPP_
#define BALSTD_DRIVERS_HPP_

#include "tap/drivers.hpp"

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
#include "aruwsrc/mock/control_operator_interface_mock.hpp"
#include "aruwsrc/mock/turret_mcb_can_comm_mock.hpp"
#include "aruwsrc/mock/vision_coprocessor_mock.hpp"
#else
#include "aruwsrc/communication/can/turret_mcb_can_comm.hpp"
#include "aruwsrc/communication/sensors/imu/ism330/ism330.hpp"
#include "aruwsrc/communication/serial/vision_coprocessor.hpp"

#include "balstd_control_operator_interface.hpp"
#endif

namespace aruwsrc::balstd
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
          turretMCBCanCommBus(this, tap::can::CanBus::CAN_BUS1),
#ifdef ENV_UNIT_TESTS
          chassisIsm330(this)
#else
          chassisIsm330(aruwsrc::communication::sensors::imu::ism330::ISM330::chipSelectFromGpio<
                        modm::platform::GpioD12>())
#endif
    {
    }

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
    testing::NiceMock<mock::ControlOperatorInterfaceMock> controlOperatorInterface;
    testing::NiceMock<mock::VisionCoprocessorMock> visionCoprocessor;
    testing::NiceMock<mock::TurretMCBCanCommMock> turretMCBCanCommBus;
    testing::NiceMock<tap::mock::Mpu6500Mock> chassisIsm330;
#else
public:
    aruwsrc::balstd::BalstdControlOperatorInterface controlOperatorInterface;
    aruwsrc::communication::serial::VisionCoprocessor visionCoprocessor;
    aruwsrc::communication::can::TurretMCBCanComm turretMCBCanCommBus;
    aruwsrc::communication::sensors::imu::ism330::ISM330 chassisIsm330;

    void init(const float mainLoopFrequency)
    {
        visionCoprocessor.initializeCV();
        turretMCBCanCommBus.init();
        mpu6500.setCalibrationSamples(4000);
        chassisIsm330.initialize(mainLoopFrequency, 0.1f, 0);
    }

    void updateIo()
    {
        visionCoprocessor.updateSerial();
        chassisIsm330.read();
    }

    void update()
    {
        turretMCBCanCommBus.sendData();
        visionCoprocessor.sendMessage();
        chassisIsm330.periodicIMUUpdate();
        checkTurretMcbDisconnection(this);
    }

private:
    bool wasTurretMcbConnected = true;
    inline void checkTurretMcbDisconnection(Drivers* drivers)
    {
        bool turretMcbConnected = drivers->turretMCBCanCommBus.isConnected();
        if (!turretMcbConnected &&
            drivers->mpu6500.getImuState() !=
                tap::communication::sensors::imu::ImuInterface::ImuState::IMU_CALIBRATING)
        {
            tap::buzzer::playNote(&drivers->pwm, 1000);
        }
        else if (turretMcbConnected && !drivers->wasTurretMcbConnected)
        {
            tap::buzzer::silenceBuzzer(&drivers->pwm);
        }
        drivers->wasTurretMcbConnected = turretMcbConnected;
    }
#endif
};  // class aruwsrc::BalstdDrivers
}  // namespace aruwsrc::balstd

#endif  // BALSTD_DRIVERS_HPP_
