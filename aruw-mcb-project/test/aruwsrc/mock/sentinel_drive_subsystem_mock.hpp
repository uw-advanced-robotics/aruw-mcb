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

#ifndef SENTINEL_DRIVE_SUBSYSTEM_MOCK_HPP_
#define SENTINEL_DRIVE_SUBSYSTEM_MOCK_HPP_

#include <gmock/gmock.h>

#include "aruwsrc/control/sentinel/drive/sentinel_drive_subsystem.hpp"

namespace aruwsrc::mock
{
class SentinelDriveSubsystemMock : public control::sentinel::drive::SentinelDriveSubsystem
{
    SentinelDriveSubsystemMock(
        aruwlib::Drivers* drivers,
        const aruwlib::algorithms::PidConfigStruct& velocityPidConfig,
        const aruwlib::control::chassis::PowerLimiterConfig& powerLimiterConfig,
        aruwlib::gpio::Digital::InputPin leftLimitSwitch = aruwlib::gpio::Digital::InputPin::A,
        aruwlib::gpio::Digital::InputPin rightLimitSwitch = aruwlib::gpio::Digital::InputPin::B,
        aruwlib::gpio::Analog::Pin currentSensorPin = aruwlib::gpio::Analog::Pin::S,
        float wheelRadius = 0,
        float gearRatio = 0,
        float railLength = 0,
        float sentinelLength = 0,
        aruwlib::motor::MotorId leftMotorId = aruwlib::motor::MOTOR1,
        aruwlib::motor::MotorId rightMotorId = aruwlib::motor::MOTOR2,
        aruwlib::can::CanBus chassisCanBus = aruwlib::can::CanBus::CAN_BUS1);
    virtual ~SentinelDriveSubsystemMock();

    MOCK_METHOD(void, initialize, (), (override));
    MOCK_METHOD(float, absolutePosition, (), (override));
    MOCK_METHOD(void, setDesiredRpm, (float), (override));
    MOCK_METHOD(void, refresh, (), (override));
    MOCK_METHOD(void, runHardwareTests, (), (override));
    MOCK_METHOD(const char*, getName, (), (override));
};
}  // namespace aruwsrc::mock

#endif  // SENTINEL_DRIVE_SUBSYSTEM_MOCK_HPP_
