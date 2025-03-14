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

#include <gtest/gtest.h>

#include "tap/drivers.hpp"
#include "tap/mock/dji_motor_mock.hpp"

#include "aruwsrc/control/turret/constants/turret_constants.hpp"
#include "aruwsrc/control/turret/turret_motor.hpp"
#include "aruwsrc/mock/turret_controller_interface_mock.hpp"

using namespace aruwsrc;
using namespace tap::mock;
using namespace tap::algorithms;
using namespace tap::can;
using namespace tap::motor;
using namespace testing;
using namespace aruwsrc::control::turret;

class TurretMotorTest : public Test
{
protected:
    static constexpr TurretMotorConfig TURRET_MOTOR_CONFIG = {
        .startAngle = M_PI_2,
        .startEncoderValue = 2000,
        .minAngle = 0,
        .maxAngle = M_PI,
        .limitMotorAngles = true,
    };

    TurretMotorTest()
        : motor(&drivers, MOTOR1, CanBus::CAN_BUS1, false, "pitch"),
          turretMotor(&motor, TURRET_MOTOR_CONFIG),
          position(tap::algorithms::Angle(TURRET_MOTOR_CONFIG.startAngle))
    {
    }

    void SetUp() override
    {
        ON_CALL(motor, isMotorOnline).WillByDefault(ReturnPointee(&motorOnline));
        ON_CALL(motor.getInternalEncoder(), isOnline).WillByDefault(ReturnPointee(&motorOnline));
        ON_CALL(motor.getInternalEncoder(), getPosition).WillByDefault(ReturnPointee(&position));
    }

    void setEncoder(tap::algorithms::WrappedFloat position) { this->position = position; }

    tap::Drivers drivers;
    NiceMock<DjiMotorMock> motor;
    TurretMotor turretMotor;
    bool motorOnline = true;

private:
    tap::algorithms::WrappedFloat position;
};

TEST_F(TurretMotorTest, isOnline_reflective_of_motor_online)
{
    motorOnline = true;
    EXPECT_TRUE(turretMotor.isOnline());
    motorOnline = false;
    EXPECT_FALSE(turretMotor.isOnline());
}

TEST_F(TurretMotorTest, initialize__initializes_single_motor)
{
    EXPECT_CALL(motor, initialize);

    turretMotor.initialize();
}

TEST_F(TurretMotorTest, setChassisFrameSetpoint__limited_to_min_max_when_limit_angle_true)
{
    // Default expectations so turret assumes motors are good to go and within valid angle range
    std::vector<std::tuple<float, float>> limitedAndInputAnglePairs{
        {TURRET_MOTOR_CONFIG.startAngle, TURRET_MOTOR_CONFIG.startAngle},
        {TURRET_MOTOR_CONFIG.minAngle, TURRET_MOTOR_CONFIG.minAngle - modm::toRadian(5)},
        {TURRET_MOTOR_CONFIG.maxAngle, TURRET_MOTOR_CONFIG.maxAngle + modm::toRadian(5)}};

    for (auto [expectedAngle, inputAngle] : limitedAndInputAnglePairs)
    {
        turretMotor.setChassisFrameSetpoint(Angle(inputAngle));
        EXPECT_NEAR(0, turretMotor.getChassisFrameSetpoint().minDifference(expectedAngle), 1E-3);
    }
}

TEST_F(TurretMotorTest, setChassisFrameSetpoint__not_limited_when_limit_angles_false)
{
    TurretMotorConfig motorConfig = TURRET_MOTOR_CONFIG;
    motorConfig.limitMotorAngles = false;

    TurretMotor turretMotor(&motor, motorConfig);

    const float offsetAngle = modm::toRadian(5);
    const float pastMinAngle = motorConfig.minAngle - offsetAngle;
    const float pastMaxAngle = motorConfig.maxAngle + offsetAngle;

    std::vector<float> limitedAndInputAnglePairs{
        {motorConfig.startAngle},
        {pastMinAngle},
        {pastMaxAngle},
    };

    for (auto expectedAngle : limitedAndInputAnglePairs)
    {
        turretMotor.setChassisFrameSetpoint(Angle(expectedAngle));
        EXPECT_NEAR(0, turretMotor.getChassisFrameSetpoint().minDifference(expectedAngle), 1E-3);
    }
}

TEST_F(TurretMotorTest, getChassisFrameMeasuredAngle__returns_default_when_yaw_motor_offline)
{
    // Default expectations so turret assumes motors are good to go and within valid angle range
    motorOnline = false;

    turretMotor.updateMotorAngle();

    EXPECT_NEAR(
        TURRET_MOTOR_CONFIG.startAngle,
        turretMotor.getChassisFrameMeasuredAngle().getWrappedValue(),
        1E-3);
}

TEST_F(
    TurretMotorTest,
    getChassisFrameMeasuredAngle__returns_values_based_on_enc_position_if_yaw_motor_online)
{
    // Default expectations so turret assumes motors are good to go and within valid angle range
    std::vector<float> angleAndEncoderPairs{
        TURRET_MOTOR_CONFIG.startAngle + 0 * M_TWOPI / 8,
        TURRET_MOTOR_CONFIG.startAngle + 1 * M_TWOPI / 8,
        TURRET_MOTOR_CONFIG.startAngle + 2 * M_TWOPI / 8,
        TURRET_MOTOR_CONFIG.startAngle + 3 * M_TWOPI / 8,
        TURRET_MOTOR_CONFIG.startAngle + 4 * M_TWOPI / 8,
        TURRET_MOTOR_CONFIG.startAngle + 5 * M_TWOPI / 8,
        TURRET_MOTOR_CONFIG.startAngle + 6 * M_TWOPI / 8};

    for (auto angle : angleAndEncoderPairs)
    {
        setEncoder(tap::algorithms::Angle(angle - TURRET_MOTOR_CONFIG.startAngle));
        turretMotor.updateMotorAngle();
        EXPECT_NEAR(0.0f, turretMotor.getChassisFrameMeasuredAngle().minDifference(angle), 1E-3);
    }
}

TEST_F(TurretMotorTest, setMotorOutput__desired_output_0_when_turret_offline)
{
    motorOnline = false;

    EXPECT_CALL(motor, setDesiredOutput(0)).Times(1);

    turretMotor.setMotorOutput(1000);
}

TEST_F(
    TurretMotorTest,
    setMotorOutput__desired_output_identical_to_input_when_turret_online_and_enc_within_bounds)
{
    setEncoder(tap::algorithms::Angle(TURRET_MOTOR_CONFIG.startAngle));

    InSequence seq;
    EXPECT_CALL(motor, setDesiredOutput(1000));
    EXPECT_CALL(motor, setDesiredOutput(-1000));

    turretMotor.setMotorOutput(1000);

    turretMotor.setMotorOutput(-1000);
}

TEST_F(TurretMotorTest, setMotorOutput__desired_output_not_limited_if_equal_to_min_max_bound)
{
    InSequence seq;
    EXPECT_CALL(motor, setDesiredOutput(-1000));
    EXPECT_CALL(motor, setDesiredOutput(1000));

    // desired output negative, equal to min
    setEncoder(tap::algorithms::Angle(TURRET_MOTOR_CONFIG.minAngle));
    turretMotor.setMotorOutput(-1000);

    // desired output position, equal to max
    setEncoder(tap::algorithms::Angle(TURRET_MOTOR_CONFIG.maxAngle));
    turretMotor.updateMotorAngle();
    turretMotor.setMotorOutput(1000);
}

TEST_F(TurretMotorTest, updateMotorAngle_sets_actual_angle_back_to_start_when_offline)
{
    // Initially turret online
    setEncoder(
        tap::algorithms::Angle(TURRET_MOTOR_CONFIG.startAngle) + tap::algorithms::Angle(M_PI_4));

    turretMotor.updateMotorAngle();

    EXPECT_NE(
        TURRET_MOTOR_CONFIG.startAngle,
        turretMotor.getChassisFrameMeasuredAngle().getWrappedValue());

    // Now turret offline
    motorOnline = false;

    turretMotor.updateMotorAngle();

    EXPECT_NEAR(
        TURRET_MOTOR_CONFIG.startAngle,
        turretMotor.getChassisFrameMeasuredAngle().getWrappedValue(),
        1E-3);
}

TEST_F(TurretMotorTest, getValidMinError_small_min_max_values)
{
    TurretMotorConfig mc = {
        .startAngle = M_PI_4 + M_PI_2,
        .startEncoderValue = 0,
        .minAngle = M_PI_2,
        .maxAngle = M_PI,
        .limitMotorAngles = true,
    };
    TurretMotor tm(&motor, mc);

    std::vector<std::tuple<float, float, float>> setpointMeasurementErrorPairs = {
        {M_PI_2, M_PI_2, 0},
        {M_PI_2 + M_PI_4, M_PI_2 + M_PI_4, 0},
        {M_PI, M_PI, 0},
        {M_PI_2, M_PI, -M_PI_2},
        {M_PI, M_PI_2, M_PI_2},
        {M_PI_2, 0, M_PI_2},
        {M_PI_2, 1.5 * M_PI, -M_PI},
        {M_PI, 0, M_PI},
        {M_PI, 1.5 * M_PI, -M_PI_2},
        {M_PI, 0.1, M_PI - 0.1},
    };

    for (auto [setpoint, measurement, error] : setpointMeasurementErrorPairs)
    {
        EXPECT_NEAR(error, tm.getValidMinError(Angle(setpoint), Angle(measurement)), 1E-3);
    }
}

TEST_F(TurretMotorTest, getValidMinError_large_min_max_values)
{
    TurretMotorConfig mc = {
        .startAngle = 0,
        .startEncoderValue = 0,
        .minAngle = 0,
        .maxAngle = 1.5f * M_PI,
        .limitMotorAngles = true,
    };
    TurretMotor tm(&motor, mc);

    std::vector<std::tuple<float, float, float>> setpointMeasurementErrorPairs = {
        {0, 0, 0},
        {M_PI, M_PI, 0},
        {1.5 * M_PI, 1.5 * M_PI, 0},
        {0, M_PI - 0.1, -M_PI + 0.1},
        {0, M_PI + 0.1, -M_PI - 0.1},
        {0, M_PI_2 + M_PI_4, -M_PI_2 - M_PI_4},
        {0, M_PI_4, -M_PI_4},
        {0.1, M_PI - 0.1, -M_PI + 0.2},
        {M_PI_2 + M_PI_4, 1.5 * M_PI + 0.1, -M_PI_2 - M_PI_4 - 0.1},
        {M_PI_2 + M_PI_4, M_TWOPI - 0.1, M_PI_2 + M_PI_4 + 0.1},
        {0, 1.5 * M_PI, -1.5 * M_PI},
        {1.5 * M_PI, 0, 1.5 * M_PI},
    };

    for (auto [setpoint, measurement, error] : setpointMeasurementErrorPairs)
    {
        EXPECT_NEAR(error, tm.getValidMinError(Angle(setpoint), Angle(measurement)), 1E-3);
    }
}

TEST_F(TurretMotorTest, setChassisFrameSetpoint_large_min_max_difference_limited_correctly)
{
    TurretMotorConfig mc = {
        .startAngle = 0,
        .startEncoderValue = 0,
        .minAngle = -M_PI_2,
        .maxAngle = M_PI,
        .limitMotorAngles = true,
    };
    TurretMotor tm(&motor, mc);

    tm.setChassisFrameSetpoint(Angle(-M_PI_2 - M_PI_4 / 2.0f));
    EXPECT_NEAR(0, tm.getChassisFrameSetpoint().minDifference(-M_PI_2), 1E-3);

    tm.setChassisFrameSetpoint(Angle(M_PI + M_PI_4 / 2.0f));
    EXPECT_NEAR(0, tm.getChassisFrameSetpoint().minDifference(M_PI), 1E-3);
}

TEST_F(TurretMotorTest, getValidChassisMeasurementError_various_setpoints)
{
    TurretMotorConfig mc = {
        .startAngle = 0,
        .startEncoderValue = 0,
        .minAngle = -M_PI_2,
        .maxAngle = M_PI_2,
        .limitMotorAngles = true,
    };
    TurretMotor tm(&motor, mc);

    motorOnline = true;

    std::vector<std::tuple<float, float, float>> errorMeasurementsToTest = {
        {-M_TWOPI, -M_TWOPI, 0},
        {-M_TWOPI, 0, 0},
        {-M_TWOPI, M_TWOPI, 0},
        {M_TWOPI, -M_TWOPI, 0},
    };

    setEncoder(tap::algorithms::Angle(mc.startAngle));
    tm.updateMotorAngle();

    for (auto [measured, setpoint, expectedErr] : errorMeasurementsToTest)
    {
        setEncoder(tap::algorithms::Angle(measured));
        tm.updateMotorAngle();

        EXPECT_NEAR(0, tm.getChassisFrameMeasuredAngle().minDifference(measured), 1E-3);

        tm.setChassisFrameSetpoint(Angle(setpoint));

        EXPECT_NEAR(0, tm.getChassisFrameSetpoint().minDifference(setpoint), 1E-3);

        EXPECT_NEAR(expectedErr, tm.getValidMinError(Angle(setpoint), Angle(measured)), 1E-3);
        EXPECT_NEAR(expectedErr, tm.getValidChassisMeasurementError(), 1E-3);
    }
}
