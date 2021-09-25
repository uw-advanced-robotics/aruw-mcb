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

    ON_CALL(pitchMotor, isMotorOnline).WillByDefault(Return(false));
    ON_CALL(yawMotor, isMotorOnline).WillByDefault(Return(false));
    EXPECT_FALSE(turret.isOnline());

    ON_CALL(pitchMotor, isMotorOnline).WillByDefault(Return(true));
    ON_CALL(yawMotor, isMotorOnline).WillByDefault(Return(false));
    EXPECT_FALSE(turret.isOnline());

    ON_CALL(pitchMotor, isMotorOnline).WillByDefault(Return(false));
    ON_CALL(yawMotor, isMotorOnline).WillByDefault(Return(true));
    EXPECT_FALSE(turret.isOnline());

    ON_CALL(pitchMotor, isMotorOnline).WillByDefault(Return(true));
    ON_CALL(yawMotor, isMotorOnline).WillByDefault(Return(true));
    EXPECT_TRUE(turret.isOnline());
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
