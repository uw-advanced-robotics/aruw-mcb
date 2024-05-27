/*
 * Copyright (c) 2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef TURRET_MOTOR_MOCK_HPP_
#define TURRET_MOTOR_MOCK_HPP_

#include <gmock/gmock.h>

#include "aruwsrc/control/turret/turret_motor.hpp"

namespace aruwsrc::mock
{
class TurretMotorMock : public control::turret::TurretMotor
{
public:
    TurretMotorMock(
        tap::motor::MotorInterface *motor,
        const control::turret::TurretMotorConfig &motorConfig);
    virtual ~TurretMotorMock();

    MOCK_METHOD(void, initialize, (), (override));
    MOCK_METHOD(void, updateMotorAngle, (), (override));
    MOCK_METHOD(void, setMotorOutput, (float), (override));
    MOCK_METHOD(
        void,
        attachTurretController,
        (const control::turret::algorithms::TurretControllerInterface *),
        (override));
    MOCK_METHOD(void, setChassisFrameSetpoint, (WrappedFloat));
    MOCK_METHOD(bool, isOnline, (), (const override));
    MOCK_METHOD(WrappedFloat, getChassisFrameSetpoint, (), (const override));
    MOCK_METHOD(
        const tap::algorithms::WrappedFloat &,
        getChassisFrameMeasuredAngle,
        (),
        (const override));
    MOCK_METHOD(float, getChassisFrameVelocity, (), (const override));
    MOCK_METHOD(float, getAngleFromCenter, (), (const override));
    MOCK_METHOD(
        const control::turret::algorithms::TurretControllerInterface *,
        getTurretController,
        (),
        (const override));
    MOCK_METHOD(const control::turret::TurretMotorConfig &, getConfig, (), (const override));
    MOCK_METHOD(float, getValidChassisMeasurementError, (), (const override));
    MOCK_METHOD(
        float,
        getValidMinError,
        (const WrappedFloat, const WrappedFloat),
        (const override));

private:
    aruwsrc::control::turret::TurretMotorConfig defaultConfig;
};
}  // namespace aruwsrc::mock

#endif  //  TURRET_MOTOR_MOCK_HPP_
