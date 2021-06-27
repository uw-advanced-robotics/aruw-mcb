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

#ifndef CHASSIS_SUBSYSTEM_MOCK_HPP_
#define CHASSIS_SUBSYSTEM_MOCK_HPP_

#include <gmock/gmock.h>

#include "aruwsrc/control/chassis/chassis_subsystem.hpp"

namespace aruwsrc
{
namespace mock
{
class ChassisSubsystemMock : public aruwsrc::chassis::ChassisSubsystem
{
public:
    ChassisSubsystemMock(
        aruwlib::Drivers* drivers,
        float motorGearboxRatio = 0,
        float widthBetweenWheelsX = 0,
        float widthBetweenWheelsY = 0,
        float wheelRadius = 0,
        float maxWheelSpeedSingleMotor = 0,
        float gimbalXOffset = 0,
        float gimbalYOffset = 0,
        float chassisRevolvePidMaxP = 0,
        float chassisRevolvePidMaxD = 0,
        float chassisRevolvePidKD = 0,
        float chassisRevolvePidMaxOutput = 0,
        float minErrorRotationD = 0,
        float minRotationThreshold = 0,
        float velocityPidKp = 0,
        float velocityPidKi = 0,
        float velocityPidKd = 0,
        float velocityPidMaxErrSum = 0,
        float velocityPidMaxOutput = 0,
        float maxEnergyBuffer = 0,
        float energyBufferLimitThreshold = 0,
        float energyBufferCritThreshold = 0,
        float powerConsumptionThreshold = 0,
        float currentAllocatedForEnergyBufferLimiting = 0,
        aruwlib::can::CanBus canBus = aruwlib::can::CanBus::CAN_BUS1,
        aruwlib::motor::MotorId leftFrontMotorId = aruwlib::motor::MOTOR1,
        aruwlib::motor::MotorId leftBackMotorId = aruwlib::motor::MOTOR2,
        aruwlib::motor::MotorId rightFrontMotorId = aruwlib::motor::MOTOR3,
        aruwlib::motor::MotorId rightBackMotorId = aruwlib::motor::MOTOR4,
        aruwlib::gpio::Analog::Pin currentPin);

    virtual ~ChassisSubsystemMock();

    MOCK_METHOD(void, initialize, (), (override));
    MOCK_METHOD(void, setDesiredOutput, (float x, float y, float z), (override));
    MOCK_METHOD(float, chassisSpeedRotationPID, (float currentAngleError, float kp), (override));
    MOCK_METHOD(void, refresh, (), (override));
    MOCK_METHOD(
        float,
        calculateRotationTranslationalGain,
        (float chassisRotationDesiredWheelspeed),
        ());
    MOCK_METHOD(int16_t, getLeftFrontRpmActual, (), (const override));
    MOCK_METHOD(int16_t, getLeftBackRpmActual, (), (const override));
    MOCK_METHOD(int16_t, getRightFrontRpmActual, (), (const override));
    MOCK_METHOD(int16_t, getRightBackRpmActual, (), (const override));
    MOCK_METHOD(float, getDesiredRotation, (), (const override));
};  // class ChassisSubsystemMock
}  // namespace mock
}  // namespace aruwsrc

#endif  // CHASSIS_SUBSYSTEM_MOCK_HPP_
