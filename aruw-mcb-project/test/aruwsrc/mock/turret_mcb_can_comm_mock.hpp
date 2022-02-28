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

#ifndef IMU_RX_LISTENER_MOCK_HPP_
#define IMU_RX_LISTENER_MOCK_HPP_

#include <gmock/gmock.h>

#include "aruwsrc/communication/can/turret_mcb_can_comm.hpp"

namespace aruwsrc::mock
{
class TurretMCBCanCommMock : public can::TurretMCBCanComm
{
public:
    TurretMCBCanCommMock(aruwsrc::Drivers *drivers);
    ~TurretMCBCanCommMock();

    MOCK_METHOD(void, init, (), (override));
    MOCK_METHOD(float, getPitch, (), (const override));
    MOCK_METHOD(float, getPitchVelocity, (), (const override));
    MOCK_METHOD(float, getYaw, (), (const override));
    MOCK_METHOD(float, getYawVelocity, (), (const override));
    MOCK_METHOD(bool, getLimitSwitchDepressed, (), (const override));
    MOCK_METHOD(bool, isConnected, (), (const override));
    MOCK_METHOD(void, setOpenHopperCover, (bool), (override));
    MOCK_METHOD(void, setLaserStatus, (bool), (override));
    MOCK_METHOD(void, sendImuCalibrationRequest, (), (override));
    MOCK_METHOD(void, sendData, (), (override));
};
}  // namespace aruwsrc::mock

#endif  // IMU_RX_LISTENER_MOCK_HPP_
