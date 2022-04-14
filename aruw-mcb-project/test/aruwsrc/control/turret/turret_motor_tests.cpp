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

#include "tap/mock/dji_motor_mock.hpp"

#include "aruwsrc/control/turret/constants/turret_constants.hpp"
#include "aruwsrc/control/turret/turret_motor.hpp"
#include "aruwsrc/drivers.hpp"

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
        .maxAngle = modm::toRadian(179),
        .limitMotorAngles = true,
    };

    TurretMotorTest()
        : motor(&drivers, MOTOR1, CanBus::CAN_BUS1, false, "pitch"),
          turretMotor(&motor, TURRET_MOTOR_CONFIG)
    {
    }

    void SetUp() override
    {
        ON_CALL(motor, isMotorOnline).WillByDefault(ReturnPointee(&motorOnline));
        ON_CALL(motor, getEncoderWrapped).WillByDefault(ReturnPointee(&encoderWrapped));
        ON_CALL(motor, getEncoderUnwrapped).WillByDefault(ReturnPointee(&encoderWrapped));
    }

    Drivers drivers;
    NiceMock<DjiMotorMock> motor;
    TurretMotor turretMotor;
    bool motorOnline = true;
    uint16_t encoderWrapped = 0;
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
        turretMotor.setChassisFrameSetpoint(inputAngle);
        EXPECT_NEAR(expectedAngle, turretMotor.getChassisFrameSetpoint().getValue(), 1E-3);
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

    std::vector<std::tuple<float, float>> limitedAndInputAnglePairs{
        {motorConfig.startAngle, motorConfig.startAngle},
        {ContiguousFloat(pastMinAngle, 0, M_TWOPI).getValue(), pastMinAngle},
        {ContiguousFloat(pastMaxAngle, 0, M_TWOPI).getValue(), pastMaxAngle},
    };

    for (auto [expectedAngle, inputAngle] : limitedAndInputAnglePairs)
    {
        turretMotor.setChassisFrameSetpoint(inputAngle);
        EXPECT_NEAR(expectedAngle, turretMotor.getChassisFrameSetpoint().getValue(), 1E-3);
    }
}

TEST_F(TurretMotorTest, getChassisFrameMeasuredAngle__returns_default_when_yaw_motor_offline)
{
    // Default expectations so turret assumes motors are good to go and within valid angle range
    motorOnline = false;

    turretMotor.updateMotorAngle();

    EXPECT_NEAR(
        TURRET_MOTOR_CONFIG.startAngle,
        turretMotor.getChassisFrameMeasuredAngle().getValue(),
        1E-3);
}

TEST_F(
    TurretMotorTest,
    getChassisFrameMeasuredAngle__returns_values_based_on_enc_position_if_yaw_motor_online)
{
    // Default expectations so turret assumes motors are good to go and within valid angle range
    const int encStep = DjiMotor::ENC_RESOLUTION / 8;
    std::vector<std::tuple<float, int>> angleAndEncoderPairs{
        {M_PI_2, TURRET_MOTOR_CONFIG.startEncoderValue},
        {M_PI_2 + M_TWOPI / 8, TURRET_MOTOR_CONFIG.startEncoderValue + encStep},
        {M_PI_2 + 2 * M_TWOPI / 8, TURRET_MOTOR_CONFIG.startEncoderValue + 2 * encStep},
        {M_PI_2 + 3 * M_TWOPI / 8, TURRET_MOTOR_CONFIG.startEncoderValue + 3 * encStep},
        {M_PI_2 + 4 * M_TWOPI / 8, TURRET_MOTOR_CONFIG.startEncoderValue + 4 * encStep},
        {M_PI_2 + 5 * M_TWOPI / 8, TURRET_MOTOR_CONFIG.startEncoderValue + 5 * encStep},
        {M_PI_2 + 6 * M_TWOPI / 8, TURRET_MOTOR_CONFIG.startEncoderValue + 6 * encStep},
    };

    for (auto [angle, encoder] : angleAndEncoderPairs)
    {
        encoderWrapped = encoder % DjiMotor::ENC_RESOLUTION;
        turretMotor.updateMotorAngle();
        EXPECT_NEAR(0.0f, turretMotor.getChassisFrameMeasuredAngle().difference(angle), 1E-3);
    }
}

TEST_F(TurretMotorTest, getAngleFromCenter__return_0_when_motors_offline)
{
    motorOnline = false;

    turretMotor.updateMotorAngle();

    EXPECT_NEAR(0, turretMotor.getAngleFromCenter(), 1E-3);
}

TEST_F(TurretMotorTest, getAngleFromCenter__valid_encoder_angles)
{
    const int encStep = DjiMotor::ENC_RESOLUTION / 8;

    std::vector<std::tuple<float, int>> angleAndEncoderPairs{
        {0, TURRET_MOTOR_CONFIG.startEncoderValue},
        {M_TWOPI / 8, TURRET_MOTOR_CONFIG.startEncoderValue + encStep},
        {2 * M_TWOPI / 8, TURRET_MOTOR_CONFIG.startEncoderValue + 2 * encStep},
        {3 * M_TWOPI / 8, TURRET_MOTOR_CONFIG.startEncoderValue + 3 * encStep},
        {4 * M_TWOPI / 8, TURRET_MOTOR_CONFIG.startEncoderValue + 4 * encStep},
        {-3 * M_TWOPI / 8, TURRET_MOTOR_CONFIG.startEncoderValue + 5 * encStep},
        {-2 * M_TWOPI / 8, TURRET_MOTOR_CONFIG.startEncoderValue + 6 * encStep},
    };

    for (auto [angle, encoder] : angleAndEncoderPairs)
    {
        encoderWrapped = encoder % DjiMotor::ENC_RESOLUTION;
        turretMotor.updateMotorAngle();
        EXPECT_NEAR(angle, turretMotor.getAngleFromCenter(), 1E-3);
    }
}

TEST_F(TurretMotorTest, setMotorOutput__desired_output_0_when_turret_offline)
{
    motorOnline = false;

    EXPECT_CALL(motor, setDesiredOutput).Times(0);

    turretMotor.setMotorOutput(1000);
}

TEST_F(
    TurretMotorTest,
    setMotorOutput__desired_output_identical_to_input_when_turret_online_and_enc_within_bounds)
{
    encoderWrapped = TURRET_MOTOR_CONFIG.startEncoderValue;

    InSequence seq;
    EXPECT_CALL(motor, setDesiredOutput(1000));
    EXPECT_CALL(motor, setDesiredOutput(-1000));

    turretMotor.setMotorOutput(1000);

    turretMotor.setMotorOutput(-1000);
}

TEST_F(TurretMotorTest, setMotorOutput__desired_output_not_limited_if_equal_to_min_max_bound)
{
    uint16_t minEncoderValue =
        ContiguousFloat(
            TURRET_MOTOR_CONFIG.startEncoderValue +
                +(TURRET_MOTOR_CONFIG.minAngle - TURRET_MOTOR_CONFIG.startAngle) *
                    DjiMotor::ENC_RESOLUTION / M_TWOPI +
                1,
            0,
            DjiMotor::ENC_RESOLUTION)
            .getValue();

    uint16_t maxEncoderValue =
        ContiguousFloat(
            TURRET_MOTOR_CONFIG.startEncoderValue +
                +(TURRET_MOTOR_CONFIG.maxAngle - TURRET_MOTOR_CONFIG.startAngle) *
                    DjiMotor::ENC_RESOLUTION / M_TWOPI -
                1,
            0,
            DjiMotor::ENC_RESOLUTION)
            .getValue();

    InSequence seq;
    EXPECT_CALL(motor, setDesiredOutput(-1000));
    EXPECT_CALL(motor, setDesiredOutput(1000));

    // desired output negative, equal to min
    encoderWrapped = minEncoderValue;
    turretMotor.updateMotorAngle();
    turretMotor.setMotorOutput(-1000);

    // desired output position, equal to max
    encoderWrapped = maxEncoderValue;
    turretMotor.updateMotorAngle();
    turretMotor.setMotorOutput(1000);
}

TEST_F(
    TurretMotorTest,
    setMotorOutput__desired_output_0_if_out_of_bounds_and_input_des_output_wrong_way)
{
    uint16_t lessThanMinEncoderValue =
        ContiguousFloat(
            TURRET_MOTOR_CONFIG.startEncoderValue +
                +(TURRET_MOTOR_CONFIG.minAngle - TURRET_MOTOR_CONFIG.startAngle -
                  modm::toRadian(10)) *
                    DjiMotor::ENC_RESOLUTION / M_TWOPI,
            0,
            DjiMotor::ENC_RESOLUTION)
            .getValue();
    uint16_t greaterThanMaxEncoderValue =
        ContiguousFloat(
            TURRET_MOTOR_CONFIG.startEncoderValue +
                +(TURRET_MOTOR_CONFIG.maxAngle - TURRET_MOTOR_CONFIG.startAngle +
                  modm::toRadian(10)) *
                    DjiMotor::ENC_RESOLUTION / M_TWOPI,
            0,
            DjiMotor::ENC_RESOLUTION)
            .getValue();

    InSequence seq;
    EXPECT_CALL(motor, setDesiredOutput(0));
    EXPECT_CALL(motor, setDesiredOutput(0));

    // desired output negative, angle less than min
    encoderWrapped = lessThanMinEncoderValue;
    turretMotor.updateMotorAngle();
    turretMotor.setMotorOutput(-1000);

    // desired output position, angle greater than max
    encoderWrapped = greaterThanMaxEncoderValue;
    turretMotor.updateMotorAngle();
    turretMotor.setMotorOutput(1000);
}

TEST_F(TurretMotorTest, updateMotorAngle_sets_actual_angle_back_to_start_when_offline)
{
    // Initially turret online
    encoderWrapped = TURRET_MOTOR_CONFIG.startEncoderValue + 1000;

    turretMotor.updateMotorAngle();

    EXPECT_NE(
        TURRET_MOTOR_CONFIG.startAngle,
        turretMotor.getChassisFrameMeasuredAngle().getValue());

    // Now turret offline
    motorOnline = false;

    turretMotor.updateMotorAngle();

    EXPECT_NEAR(
        TURRET_MOTOR_CONFIG.startAngle,
        turretMotor.getChassisFrameMeasuredAngle().getValue(),
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
        tm.setChassisFrameSetpoint(setpoint);
        EXPECT_NEAR(error, tm.getValidMinError(measurement), 1E-3);
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
        {M_PI_2 + M_PI_4, M_TWOPI - 0.1, -M_PI - M_PI_4 + 0.1},
        {0, 1.5 * M_PI, -1.5 * M_PI},
        {1.5 * M_PI, 0, 1.5 * M_PI},
    };

    for (auto [setpoint, measurement, error] : setpointMeasurementErrorPairs)
    {
        tm.setChassisFrameSetpoint(setpoint);
        EXPECT_NEAR(error, tm.getValidMinError(measurement), 1E-3);
    }
}

TEST_F(TurretMotorTest, getValidMinError_large_swapped_min_max_values) {}
