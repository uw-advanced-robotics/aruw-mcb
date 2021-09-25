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

#include "aruwsrc/control/turret/turret_subsystem.hpp"

using namespace tap;
using namespace tap::mock;
using namespace tap::can;
using namespace tap::motor;
using namespace testing;
using namespace aruwsrc::control::turret;

#define CONSTRUCT_SHARED_TEST_OBJECTS()                                                    \
    Drivers drivers;                                                                       \
    NiceMock<DjiMotorMock> pitchMotor(&drivers, MOTOR1, CanBus::CAN_BUS1, false, "pitch"); \
    NiceMock<DjiMotorMock> yawMotor(&drivers, MOTOR2, CanBus::CAN_BUS1, false, "yaw");

static void setEncoderWrapped(DjiMotorMock &motor, uint16_t encoder)
{
    ON_CALL(motor, getEncoderWrapped).WillByDefault(Return(encoder));
}

TEST(TurretSubsystem, initialize__initializes_both_motors)
{
    CONSTRUCT_SHARED_TEST_OBJECTS();
    TurretSubsystem turret(&drivers, &pitchMotor, &yawMotor);

    EXPECT_CALL(pitchMotor, initialize);
    EXPECT_CALL(yawMotor, initialize);

    turret.initialize();
}

TEST(TurretSubsystem, setYawSetpoint__limited_to_min_max_when_limitYaw_false)
{
    CONSTRUCT_SHARED_TEST_OBJECTS();
    TurretSubsystem turret(&drivers, &pitchMotor, &yawMotor);

    // Default expectations so turret assumes motors are good to go and within valid angle range
    ON_CALL(pitchMotor, isMotorOnline).WillByDefault(Return(false));
    ON_CALL(yawMotor, isMotorOnline).WillByDefault(Return(true));

    std::vector<std::tuple<float, float>> limitedAndInputAnglePairs{
        {TurretSubsystem::YAW_START_ANGLE, TurretSubsystem::YAW_START_ANGLE},
        {TurretSubsystem::YAW_MIN_ANGLE, TurretSubsystem::YAW_MIN_ANGLE - 5},
        {TurretSubsystem::YAW_MAX_ANGLE, TurretSubsystem::YAW_MAX_ANGLE + 5}};

    for (auto [expectedAngle, inputAngle] : limitedAndInputAnglePairs)
    {
        turret.setYawSetpoint(inputAngle);
        EXPECT_FLOAT_EQ(expectedAngle, turret.getYawSetpoint());
    }
}

TEST(TurretSubsystem, setYawSetpoint__not_limited_when_limitYaw_true)
{
    CONSTRUCT_SHARED_TEST_OBJECTS();
    TurretSubsystem turret(&drivers, &pitchMotor, &yawMotor, false);

    // Default expectations so turret assumes motors are good to go and within valid angle range
    ON_CALL(pitchMotor, isMotorOnline).WillByDefault(Return(false));
    ON_CALL(yawMotor, isMotorOnline).WillByDefault(Return(true));

    std::vector<std::tuple<float, float>> limitedAndInputAnglePairs{
        {TurretSubsystem::YAW_START_ANGLE, TurretSubsystem::YAW_START_ANGLE},
        {tap::algorithms::ContiguousFloat(TurretSubsystem::YAW_MIN_ANGLE - 5, 0, 360).getValue(),
         TurretSubsystem::YAW_MIN_ANGLE - 5},
        {tap::algorithms::ContiguousFloat(TurretSubsystem::YAW_MAX_ANGLE + 5, 0, 360).getValue(),
         TurretSubsystem::YAW_MAX_ANGLE + 5}};

    for (auto [expectedAngle, inputAngle] : limitedAndInputAnglePairs)
    {
        turret.setYawSetpoint(inputAngle);
        EXPECT_FLOAT_EQ(expectedAngle, turret.getYawSetpoint());
    }
}

TEST(TurretSubsystem, setPitchSetpoint__limited_to_min_max)
{
    CONSTRUCT_SHARED_TEST_OBJECTS();
    TurretSubsystem turret(&drivers, &pitchMotor, &yawMotor);

    // Default expectations so turret assumes motors are good to go and within valid angle range
    ON_CALL(pitchMotor, isMotorOnline).WillByDefault(Return(true));
    ON_CALL(yawMotor, isMotorOnline).WillByDefault(Return(false));

    std::vector<std::tuple<float, float>> limitedAndInputAnglePairs{
        {TurretSubsystem::PITCH_START_ANGLE, TurretSubsystem::PITCH_START_ANGLE},
        {TurretSubsystem::PITCH_MIN_ANGLE, TurretSubsystem::PITCH_MIN_ANGLE - 5},
        {TurretSubsystem::PITCH_MAX_ANGLE, TurretSubsystem::PITCH_MAX_ANGLE + 5}};

    for (auto [expectedAngle, inputAngle] : limitedAndInputAnglePairs)
    {
        turret.setPitchSetpoint(inputAngle);
        EXPECT_FLOAT_EQ(expectedAngle, turret.getPitchSetpoint());
    }
}

TEST(TurretSubsystem, getCurrentYawValue__returns_default_when_yaw_motor_offline)
{
    CONSTRUCT_SHARED_TEST_OBJECTS();
    TurretSubsystem turret(&drivers, &pitchMotor, &yawMotor);

    // Default expectations so turret assumes motors are good to go and within valid angle range
    ON_CALL(pitchMotor, isMotorOnline).WillByDefault(Return(false));
    ON_CALL(yawMotor, isMotorOnline).WillByDefault(Return(false));

    turret.refresh();

    EXPECT_FLOAT_EQ(TurretSubsystem::YAW_START_ANGLE, turret.getCurrentYawValue().getValue());
}

TEST(TurretSubsystem, getCurrentYawValue__returns_values_based_on_enc_postiion_if_yaw_motor_online)
{
    CONSTRUCT_SHARED_TEST_OBJECTS();
    TurretSubsystem turret(&drivers, &pitchMotor, &yawMotor);

    // Default expectations so turret assumes motors are good to go and within valid angle range
    ON_CALL(pitchMotor, isMotorOnline).WillByDefault(Return(false));
    ON_CALL(yawMotor, isMotorOnline).WillByDefault(Return(true));

    std::vector<std::tuple<float, int>> angleAndEncoderPairs{
        {90, TurretSubsystem::YAW_START_ENCODER_POSITION},
        {135, TurretSubsystem::YAW_START_ENCODER_POSITION + DjiMotor::ENC_RESOLUTION / 8},
        {180, TurretSubsystem::YAW_START_ENCODER_POSITION + 2 * DjiMotor::ENC_RESOLUTION / 8},
        {225, TurretSubsystem::YAW_START_ENCODER_POSITION + 3 * DjiMotor::ENC_RESOLUTION / 8},
        {270, TurretSubsystem::YAW_START_ENCODER_POSITION + 4 * DjiMotor::ENC_RESOLUTION / 8},
        {315, TurretSubsystem::YAW_START_ENCODER_POSITION + 5 * DjiMotor::ENC_RESOLUTION / 8},
        {0, TurretSubsystem::YAW_START_ENCODER_POSITION + 6 * DjiMotor::ENC_RESOLUTION / 8}};

    for (auto [angle, encoder] : angleAndEncoderPairs)
    {
        setEncoderWrapped(yawMotor, encoder % DjiMotor::ENC_RESOLUTION);
        turret.refresh();
        EXPECT_FLOAT_EQ(angle, turret.getCurrentYawValue().getValue());
    }
}

TEST(TurretSubsystem, getCurrentPitchValue__returns_default_when_pitch_motor_offline)
{
    CONSTRUCT_SHARED_TEST_OBJECTS();
    TurretSubsystem turret(&drivers, &pitchMotor, &yawMotor);

    // Default expectations so turret assumes motors are good to go and within valid angle range
    ON_CALL(pitchMotor, isMotorOnline).WillByDefault(Return(false));
    ON_CALL(yawMotor, isMotorOnline).WillByDefault(Return(false));

    turret.refresh();

    EXPECT_FLOAT_EQ(TurretSubsystem::PITCH_START_ANGLE, turret.getCurrentPitchValue().getValue());
}

TEST(
    TurretSubsystem,
    getCurrentPitchValue__returns_values_based_on_enc_postiion_if_pitch_motor_online)
{
    CONSTRUCT_SHARED_TEST_OBJECTS();
    TurretSubsystem turret(&drivers, &pitchMotor, &yawMotor);

    // Default expectations so turret assumes motors are good to go and within valid angle range
    ON_CALL(pitchMotor, isMotorOnline).WillByDefault(Return(true));
    ON_CALL(yawMotor, isMotorOnline).WillByDefault(Return(false));

    std::vector<std::tuple<float, int>> angleAndEncoderPairs{
        {90, TurretSubsystem::PITCH_START_ENCODER_POSITION},
        {135, TurretSubsystem::PITCH_START_ENCODER_POSITION + DjiMotor::ENC_RESOLUTION / 8},
        {180, TurretSubsystem::PITCH_START_ENCODER_POSITION + 2 * DjiMotor::ENC_RESOLUTION / 8},
        {225, TurretSubsystem::PITCH_START_ENCODER_POSITION + 3 * DjiMotor::ENC_RESOLUTION / 8},
        {270, TurretSubsystem::PITCH_START_ENCODER_POSITION + 4 * DjiMotor::ENC_RESOLUTION / 8},
        {315, TurretSubsystem::PITCH_START_ENCODER_POSITION + 5 * DjiMotor::ENC_RESOLUTION / 8},
        {0, TurretSubsystem::PITCH_START_ENCODER_POSITION + 6 * DjiMotor::ENC_RESOLUTION / 8}};

    for (auto [angle, encoder] : angleAndEncoderPairs)
    {
        setEncoderWrapped(pitchMotor, encoder % DjiMotor::ENC_RESOLUTION);
        turret.refresh();
        EXPECT_FLOAT_EQ(angle, turret.getCurrentPitchValue().getValue());
    }
}

TEST(TurretSubsystem, onHardwareTestStart__sets_des_out_0)
{
    CONSTRUCT_SHARED_TEST_OBJECTS();
    TurretSubsystem turret(&drivers, &pitchMotor, &yawMotor);

    // Default expectations so turret assumes motors are good to go and within valid angle range
    ON_CALL(pitchMotor, isMotorOnline).WillByDefault(Return(true));
    ON_CALL(yawMotor, isMotorOnline).WillByDefault(Return(true));
    ON_CALL(pitchMotor, getEncoderWrapped)
        .WillByDefault(Return(TurretSubsystem::PITCH_START_ENCODER_POSITION));
    ON_CALL(yawMotor, getEncoderWrapped)
        .WillByDefault(Return(TurretSubsystem::YAW_START_ENCODER_POSITION));
    turret.refresh();

    // Make sure desired output isn't 0
    EXPECT_CALL(pitchMotor, setDesiredOutput(Ne(0)));
    EXPECT_CALL(yawMotor, setDesiredOutput(Ne(0)));

    turret.setYawMotorOutput(100);
    turret.setPitchMotorOutput(100);

    EXPECT_CALL(pitchMotor, setDesiredOutput(0));
    EXPECT_CALL(yawMotor, setDesiredOutput(0));

    turret.onHardwareTestStart();
}

TEST(TurretSubsystem, isOnline__return_true_when_all_motors_online)
{
    CONSTRUCT_SHARED_TEST_OBJECTS();
    TurretSubsystem turret(&drivers, &pitchMotor, &yawMotor);

    ON_CALL(pitchMotor, isMotorOnline).WillByDefault(Return(true));
    ON_CALL(yawMotor, isMotorOnline).WillByDefault(Return(true));
    EXPECT_TRUE(turret.isOnline());
}

TEST(TurretSubsystem, isOnline__return_false_when_some_motors_offline)
{
    CONSTRUCT_SHARED_TEST_OBJECTS();
    TurretSubsystem turret(&drivers, &pitchMotor, &yawMotor);

    ON_CALL(pitchMotor, isMotorOnline).WillByDefault(Return(false));
    ON_CALL(yawMotor, isMotorOnline).WillByDefault(Return(false));
    EXPECT_FALSE(turret.isOnline());

    ON_CALL(pitchMotor, isMotorOnline).WillByDefault(Return(true));
    ON_CALL(yawMotor, isMotorOnline).WillByDefault(Return(false));
    EXPECT_FALSE(turret.isOnline());

    ON_CALL(pitchMotor, isMotorOnline).WillByDefault(Return(false));
    ON_CALL(yawMotor, isMotorOnline).WillByDefault(Return(true));
    EXPECT_FALSE(turret.isOnline());
}

TEST(TurretSubsystem, getPitchAngleFromCenter__return_0_when_motors_offline)
{
    CONSTRUCT_SHARED_TEST_OBJECTS();
    TurretSubsystem turret(&drivers, &pitchMotor, &yawMotor);

    ON_CALL(yawMotor, isMotorOnline).WillByDefault(Return(false));
    ON_CALL(pitchMotor, isMotorOnline).WillByDefault(Return(false));

    turret.refresh();

    EXPECT_FLOAT_EQ(0, turret.getPitchAngleFromCenter());
}

TEST(TurretSubsystem, getPitchAngleFromCenter__valid_encoder_angles)
{
    CONSTRUCT_SHARED_TEST_OBJECTS();
    TurretSubsystem turret(&drivers, &pitchMotor, &yawMotor);

    ON_CALL(yawMotor, isMotorOnline).WillByDefault(Return(true));
    ON_CALL(pitchMotor, isMotorOnline).WillByDefault(Return(true));

    std::vector<std::tuple<float, int>> angleAndEncoderPairs{
        {0, TurretSubsystem::PITCH_START_ENCODER_POSITION},
        {45, TurretSubsystem::PITCH_START_ENCODER_POSITION + DjiMotor::ENC_RESOLUTION / 8},
        {90, TurretSubsystem::PITCH_START_ENCODER_POSITION + 2 * DjiMotor::ENC_RESOLUTION / 8},
        {135, TurretSubsystem::PITCH_START_ENCODER_POSITION + 3 * DjiMotor::ENC_RESOLUTION / 8},
        {180, TurretSubsystem::PITCH_START_ENCODER_POSITION + 4 * DjiMotor::ENC_RESOLUTION / 8},
        {-135, TurretSubsystem::PITCH_START_ENCODER_POSITION + 5 * DjiMotor::ENC_RESOLUTION / 8},
        {-90, TurretSubsystem::PITCH_START_ENCODER_POSITION + 6 * DjiMotor::ENC_RESOLUTION / 8}};

    for (auto [angle, encoder] : angleAndEncoderPairs)
    {
        setEncoderWrapped(pitchMotor, encoder % DjiMotor::ENC_RESOLUTION);
        turret.refresh();
        EXPECT_FLOAT_EQ(angle, turret.getPitchAngleFromCenter());
    }
}

TEST(TurretSubsystem, getYawAngleFromCenter__return_0_when_motors_offline)
{
    CONSTRUCT_SHARED_TEST_OBJECTS();
    TurretSubsystem turret(&drivers, &pitchMotor, &yawMotor);

    ON_CALL(yawMotor, isMotorOnline).WillByDefault(Return(false));
    ON_CALL(pitchMotor, isMotorOnline).WillByDefault(Return(false));

    turret.refresh();

    EXPECT_FLOAT_EQ(0, turret.getYawAngleFromCenter());
}

TEST(TurretSubsystem, getYawAngleFromCenter__chassisFrontBackIdentical_false)
{
    CONSTRUCT_SHARED_TEST_OBJECTS();
    TurretSubsystem turret(&drivers, &pitchMotor, &yawMotor);

    ON_CALL(yawMotor, isMotorOnline).WillByDefault(Return(true));
    ON_CALL(pitchMotor, isMotorOnline).WillByDefault(Return(true));

    std::vector<std::tuple<float, int>> angleFromCenterEncoderPairs{
        {0, TurretSubsystem::YAW_START_ENCODER_POSITION},
        {45, TurretSubsystem::YAW_START_ENCODER_POSITION + DjiMotor::ENC_RESOLUTION / 8},
        {90, TurretSubsystem::YAW_START_ENCODER_POSITION + 2 * DjiMotor::ENC_RESOLUTION / 8},
        {135, TurretSubsystem::YAW_START_ENCODER_POSITION + 3 * DjiMotor::ENC_RESOLUTION / 8},
        {180, TurretSubsystem::YAW_START_ENCODER_POSITION + 4 * DjiMotor::ENC_RESOLUTION / 8},
        {-135, TurretSubsystem::YAW_START_ENCODER_POSITION + 5 * DjiMotor::ENC_RESOLUTION / 8},
        {-90, TurretSubsystem::YAW_START_ENCODER_POSITION + 6 * DjiMotor::ENC_RESOLUTION / 8}};

    for (auto [angle, encoder] : angleFromCenterEncoderPairs)
    {
        setEncoderWrapped(yawMotor, encoder % DjiMotor::ENC_RESOLUTION);
        turret.refresh();
        EXPECT_FLOAT_EQ(angle, turret.getYawAngleFromCenter());
    }
}

TEST(TurretSubsystem, getYawAngleFromCenter__chassisFrontBackIdentical_true_limitYaw_true)
{
    CONSTRUCT_SHARED_TEST_OBJECTS();
    TurretSubsystem turret(&drivers, &pitchMotor, &yawMotor, true, true);

    ON_CALL(yawMotor, isMotorOnline).WillByDefault(Return(true));
    ON_CALL(pitchMotor, isMotorOnline).WillByDefault(Return(true));

    std::vector<std::tuple<float, int>> angleFromCenterEncoderPairs{
        {180, TurretSubsystem::YAW_START_ENCODER_POSITION + 4 * DjiMotor::ENC_RESOLUTION / 8}};

    for (auto [angle, encoder] : angleFromCenterEncoderPairs)
    {
        setEncoderWrapped(yawMotor, encoder % DjiMotor::ENC_RESOLUTION);
        turret.refresh();
        EXPECT_FLOAT_EQ(angle, turret.getYawAngleFromCenter());
    }
}

TEST(TurretSubsystem, getYawAngleFromCenter__chassisFrontBackIdentical_true_limitYaw_false)
{
    CONSTRUCT_SHARED_TEST_OBJECTS();
    TurretSubsystem turret(&drivers, &pitchMotor, &yawMotor, false, true);

    ON_CALL(yawMotor, isMotorOnline).WillByDefault(Return(true));
    ON_CALL(pitchMotor, isMotorOnline).WillByDefault(Return(true));

    std::vector<std::tuple<float, int>> angleFromCenterEncoderPairs{
        {0, TurretSubsystem::YAW_START_ENCODER_POSITION},
        {45, TurretSubsystem::YAW_START_ENCODER_POSITION + DjiMotor::ENC_RESOLUTION / 8},
        {90, TurretSubsystem::YAW_START_ENCODER_POSITION + 2 * DjiMotor::ENC_RESOLUTION / 8},
        {-45, TurretSubsystem::YAW_START_ENCODER_POSITION + 3 * DjiMotor::ENC_RESOLUTION / 8},
        {0, TurretSubsystem::YAW_START_ENCODER_POSITION + 4 * DjiMotor::ENC_RESOLUTION / 8},
        {45, TurretSubsystem::YAW_START_ENCODER_POSITION + 5 * DjiMotor::ENC_RESOLUTION / 8},
        {-90, TurretSubsystem::YAW_START_ENCODER_POSITION + 6 * DjiMotor::ENC_RESOLUTION / 8}};

    for (auto [angle, encoder] : angleFromCenterEncoderPairs)
    {
        setEncoderWrapped(yawMotor, encoder % DjiMotor::ENC_RESOLUTION);
        turret.refresh();
        EXPECT_FLOAT_EQ(angle, turret.getYawAngleFromCenter());
    }
}

TEST(TurretSubsystem, yawLimited__return_matches_constructor_value)
{
    CONSTRUCT_SHARED_TEST_OBJECTS();
    TurretSubsystem t1(&drivers, &pitchMotor, &yawMotor);
    TurretSubsystem t2(&drivers, &pitchMotor, &yawMotor, false);

    EXPECT_TRUE(t1.yawLimited());
    EXPECT_FALSE(t2.yawLimited());
}

TEST(TurretSubsystem, setPitchMotorOutput__desired_output_0_when_turret_offline)
{
    CONSTRUCT_SHARED_TEST_OBJECTS();
    TurretSubsystem turret(&drivers, &pitchMotor, &yawMotor);

    ON_CALL(pitchMotor, isMotorOnline).WillByDefault(Return(false));
    EXPECT_CALL(pitchMotor, setDesiredOutput).Times(0);

    turret.setPitchMotorOutput(1000);
}

TEST(
    TurretSubsystem,
    setPitchMotorOutput__desired_output_identical_to_input_when_turret_online_and_enc_within_bounds)
{
    CONSTRUCT_SHARED_TEST_OBJECTS();
    TurretSubsystem turret(&drivers, &pitchMotor, &yawMotor);

    ON_CALL(pitchMotor, isMotorOnline).WillByDefault(Return(true));
    ON_CALL(pitchMotor, getEncoderWrapped)
        .WillByDefault(Return(TurretSubsystem::PITCH_START_ENCODER_POSITION));

    EXPECT_CALL(pitchMotor, setDesiredOutput(1000));
    turret.setPitchMotorOutput(1000);

    EXPECT_CALL(pitchMotor, setDesiredOutput(-1000));
    turret.setPitchMotorOutput(-1000);
}

TEST(TurretSubsystem, setPitchMotorOutput__desired_output_not_limited_if_equal_to_min_max_bound)
{
    CONSTRUCT_SHARED_TEST_OBJECTS();
    TurretSubsystem turret(&drivers, &pitchMotor, &yawMotor);

    ON_CALL(pitchMotor, isMotorOnline).WillByDefault(Return(true));

    uint16_t minEncoderValue =
        tap::algorithms::ContiguousFloat(
            TurretSubsystem::PITCH_START_ENCODER_POSITION +
                +(TurretSubsystem::PITCH_MIN_ANGLE - TurretSubsystem::PITCH_START_ANGLE) *
                    DjiMotor::ENC_RESOLUTION / 360.0f +
                1,
            0,
            DjiMotor::ENC_RESOLUTION)
            .getValue();

    uint16_t maxEncoderValue =
        tap::algorithms::ContiguousFloat(
            TurretSubsystem::PITCH_START_ENCODER_POSITION +
                +(TurretSubsystem::PITCH_MAX_ANGLE - TurretSubsystem::PITCH_START_ANGLE) *
                    DjiMotor::ENC_RESOLUTION / 360.0f -
                1,
            0,
            DjiMotor::ENC_RESOLUTION)
            .getValue();

    // desired output negative, equal to min
    ON_CALL(pitchMotor, getEncoderWrapped).WillByDefault(Return(minEncoderValue));
    EXPECT_CALL(pitchMotor, setDesiredOutput(-1000));
    turret.refresh();
    turret.setPitchMotorOutput(-1000);

    // desired output position, equal to max
    ON_CALL(pitchMotor, getEncoderWrapped).WillByDefault(Return(maxEncoderValue));
    EXPECT_CALL(pitchMotor, setDesiredOutput(1000));
    turret.refresh();
    turret.setPitchMotorOutput(1000);
}

TEST(
    TurretSubsystem,
    setPitchMotorOutput__desired_output_0_if_out_of_bounds_and_input_des_output_wrong_way)
{
    CONSTRUCT_SHARED_TEST_OBJECTS();
    TurretSubsystem turret(&drivers, &pitchMotor, &yawMotor);

    ON_CALL(pitchMotor, isMotorOnline).WillByDefault(Return(true));

    uint16_t lessThanMinEncoderValue =
        tap::algorithms::ContiguousFloat(
            TurretSubsystem::PITCH_START_ENCODER_POSITION +
                +(TurretSubsystem::PITCH_MIN_ANGLE - TurretSubsystem::PITCH_START_ANGLE - 1) *
                    DjiMotor::ENC_RESOLUTION / 360.0f,
            0,
            DjiMotor::ENC_RESOLUTION)
            .getValue();

    uint16_t greaterThanMaxEncoderValue =
        tap::algorithms::ContiguousFloat(
            TurretSubsystem::PITCH_START_ENCODER_POSITION +
                +(TurretSubsystem::PITCH_MAX_ANGLE - TurretSubsystem::PITCH_START_ANGLE + 1) *
                    DjiMotor::ENC_RESOLUTION / 360.0f,
            0,
            DjiMotor::ENC_RESOLUTION)
            .getValue();

    // desired output negative, angle less than min
    ON_CALL(pitchMotor, getEncoderWrapped).WillByDefault(Return(lessThanMinEncoderValue));
    EXPECT_CALL(pitchMotor, setDesiredOutput(0));
    turret.refresh();
    turret.setPitchMotorOutput(-1000);

    // desired output position, angle greater than max
    ON_CALL(pitchMotor, getEncoderWrapped).WillByDefault(Return(greaterThanMaxEncoderValue));
    EXPECT_CALL(pitchMotor, setDesiredOutput(0));
    turret.refresh();
    turret.setPitchMotorOutput(1000);
}

TEST(TurretSubsystem, setYawMotorOutput__desired_output_0_when_turret_offline)
{
    CONSTRUCT_SHARED_TEST_OBJECTS();
    TurretSubsystem turret(&drivers, &pitchMotor, &yawMotor);

    ON_CALL(yawMotor, isMotorOnline).WillByDefault(Return(false));
    EXPECT_CALL(yawMotor, setDesiredOutput).Times(0);

    turret.setYawMotorOutput(1000);
}

TEST(
    TurretSubsystem,
    setYawMotorOutput__desired_output_identical_to_input_when_turret_online_and_enc_within_bounds)
{
    CONSTRUCT_SHARED_TEST_OBJECTS();
    TurretSubsystem turret(&drivers, &pitchMotor, &yawMotor);

    ON_CALL(yawMotor, isMotorOnline).WillByDefault(Return(true));
    ON_CALL(yawMotor, getEncoderWrapped)
        .WillByDefault(Return(TurretSubsystem::YAW_START_ENCODER_POSITION));

    turret.refresh();

    EXPECT_CALL(yawMotor, setDesiredOutput(1000));
    turret.setYawMotorOutput(1000);

    EXPECT_CALL(yawMotor, setDesiredOutput(-1000));
    turret.setYawMotorOutput(-1000);
}

TEST(TurretSubsystem, setYawMotorOutput__desired_output_not_limited_if_equal_to_min_max_bound)
{
    CONSTRUCT_SHARED_TEST_OBJECTS();
    TurretSubsystem turret(&drivers, &pitchMotor, &yawMotor);

    ON_CALL(yawMotor, isMotorOnline).WillByDefault(Return(true));

    uint16_t minEncoderValue =
        tap::algorithms::ContiguousFloat(
            TurretSubsystem::YAW_START_ENCODER_POSITION +
                +(TurretSubsystem::YAW_MIN_ANGLE - TurretSubsystem::YAW_START_ANGLE) *
                    DjiMotor::ENC_RESOLUTION / 360.0f +
                1,
            0,
            DjiMotor::ENC_RESOLUTION)
            .getValue();

    uint16_t maxEncoderValue =
        tap::algorithms::ContiguousFloat(
            TurretSubsystem::YAW_START_ENCODER_POSITION +
                +(TurretSubsystem::YAW_MAX_ANGLE - TurretSubsystem::YAW_START_ANGLE) *
                    DjiMotor::ENC_RESOLUTION / 360.0f -
                1,
            0,
            DjiMotor::ENC_RESOLUTION)
            .getValue();

    // desired output negative, angle equal to min
    ON_CALL(yawMotor, getEncoderWrapped).WillByDefault(Return(minEncoderValue));
    EXPECT_CALL(yawMotor, setDesiredOutput(-1000));
    turret.refresh();
    turret.setYawMotorOutput(-1000);

    // desired output position, angle equal to max
    ON_CALL(yawMotor, getEncoderWrapped).WillByDefault(Return(maxEncoderValue));
    EXPECT_CALL(yawMotor, setDesiredOutput(1000));
    turret.refresh();
    turret.setYawMotorOutput(1000);
}

TEST(
    TurretSubsystem,
    setYawMotorOutput__desired_output_0_if_out_of_bounds_and_input_des_output_wrong_way)
{
    CONSTRUCT_SHARED_TEST_OBJECTS();
    TurretSubsystem turret(&drivers, &pitchMotor, &yawMotor);

    ON_CALL(yawMotor, isMotorOnline).WillByDefault(Return(true));

    uint16_t lessThanMinEncoderValue =
        tap::algorithms::ContiguousFloat(
            TurretSubsystem::YAW_START_ENCODER_POSITION +
                +(TurretSubsystem::YAW_MIN_ANGLE - 1 - TurretSubsystem::YAW_START_ANGLE) *
                    DjiMotor::ENC_RESOLUTION / 360.0f,
            0,
            DjiMotor::ENC_RESOLUTION)
            .getValue();

    uint16_t greaterThanMaxEncoderValue =
        tap::algorithms::ContiguousFloat(
            TurretSubsystem::YAW_START_ENCODER_POSITION +
                +(TurretSubsystem::YAW_MAX_ANGLE + 1 - TurretSubsystem::YAW_START_ANGLE) *
                    DjiMotor::ENC_RESOLUTION / 360.0f,
            0,
            DjiMotor::ENC_RESOLUTION)
            .getValue();

    // desired output negative, angle less than min
    ON_CALL(yawMotor, getEncoderWrapped).WillByDefault(Return(lessThanMinEncoderValue));
    EXPECT_CALL(yawMotor, setDesiredOutput(0));
    turret.refresh();
    turret.setYawMotorOutput(-1000);

    // desired output position, angle greater than max
    ON_CALL(yawMotor, getEncoderWrapped).WillByDefault(Return(greaterThanMaxEncoderValue));
    EXPECT_CALL(yawMotor, setDesiredOutput(0));
    turret.refresh();
    turret.setYawMotorOutput(1000);
}

TEST(TurretSubsystem, turret_actual_angles_update_when_subsystem_in_scheduler)
{
    CONSTRUCT_SHARED_TEST_OBJECTS();
    tap::control::CommandScheduler scheduler(&drivers, true);
    TurretSubsystem turret(&drivers, &pitchMotor, &yawMotor);

    ON_CALL(yawMotor, isMotorOnline).WillByDefault(Return(true));
    ON_CALL(pitchMotor, isMotorOnline).WillByDefault(Return(true));
    EXPECT_CALL(yawMotor, getEncoderWrapped).WillOnce(Return(100));
    EXPECT_CALL(pitchMotor, getEncoderWrapped).WillOnce(Return(100));

    scheduler.registerSubsystem(&turret);
    scheduler.run();
}

TEST(TurretSubsystem, refresh_sets_actual_angle_back_to_start_when_offline)
{
    CONSTRUCT_SHARED_TEST_OBJECTS();
    TurretSubsystem turret(&drivers, &pitchMotor, &yawMotor);

    // Initially turret online
    ON_CALL(yawMotor, isMotorOnline).WillByDefault(Return(true));
    ON_CALL(pitchMotor, isMotorOnline).WillByDefault(Return(true));
    ON_CALL(yawMotor, getEncoderWrapped)
        .WillByDefault(Return(TurretSubsystem::YAW_START_ENCODER_POSITION + 1000));
    ON_CALL(pitchMotor, getEncoderWrapped)
        .WillByDefault(Return(TurretSubsystem::PITCH_START_ENCODER_POSITION + 1000));

    turret.refresh();

    EXPECT_NE(TurretSubsystem::YAW_START_ANGLE, turret.getCurrentYawValue().getValue());
    EXPECT_NE(TurretSubsystem::PITCH_START_ANGLE, turret.getCurrentPitchValue().getValue());

    // Now turret offline
    ON_CALL(yawMotor, isMotorOnline).WillByDefault(Return(false));
    ON_CALL(pitchMotor, isMotorOnline).WillByDefault(Return(false));

    turret.refresh();

    EXPECT_FLOAT_EQ(TurretSubsystem::YAW_START_ANGLE, turret.getCurrentYawValue().getValue());
    EXPECT_FLOAT_EQ(TurretSubsystem::PITCH_START_ANGLE, turret.getCurrentPitchValue().getValue());
}
