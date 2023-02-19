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

#ifndef AGITATOR_SUBSYSTEM_MOCK_HPP_
#define AGITATOR_SUBSYSTEM_MOCK_HPP_

#include <gmock/gmock.h>

#include "aruwsrc/control/agitator/agitator_subsystem.hpp"

namespace aruwsrc
{
namespace mock
{
class AgitatorSubsystemMock : public agitator::AgitatorSubsystem
{
public:
    AgitatorSubsystemMock(
        tap::Drivers* drivers,
        const tap::algorithms::SmoothPidConfig& pidConfig = tap::algorithms::SmoothPidConfig(),
        float agitatorGearRatio = 0,
        tap::motor::MotorId agitatorMotorId = tap::motor::MOTOR1,
        tap::can::CanBus agitatorCanBusId = tap::can::CanBus::CAN_BUS1,
        bool isAgitatorInverted = false,
        float jammingDistance = 0,
        uint32_t jammingTime = 0,
        bool jamLogicEnabled = false);
    virtual ~AgitatorSubsystemMock();

    MOCK_METHOD(void, initialize, (), (override));

    MOCK_METHOD(void, refresh, (), (override));

    MOCK_METHOD(void, setSetpoint, (float newAngle), (override));

    MOCK_METHOD(float, getCurrentValue, (), (const, override));

    MOCK_METHOD(float, getSetpoint, (), (const, override));

    MOCK_METHOD(bool, calibrateHere, (), (override));

    MOCK_METHOD(bool, isJammed, (), (override));

    MOCK_METHOD(bool, isCalibrated, (), (override));

    MOCK_METHOD(bool, isOnline, (), (override));

    MOCK_METHOD(float, getVelocity, (), (override));

    MOCK_METHOD(const char*, getName, (), (override));
};  // class AgitatorSubsystemMock

}  // namespace mock

}  // namespace aruwsrc

#endif  // AGITATOR_SUBSYSTEM_MOCK_HPP_
