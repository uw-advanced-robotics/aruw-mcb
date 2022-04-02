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

#include "aruwsrc/control/turret/constants/turret_controller_constants.hpp"
#include "aruwsrc/control/turret/turret_subsystem.hpp"
#include "aruwsrc/drivers.hpp"

using namespace aruwsrc;
using namespace tap::mock;
using namespace tap::can;
using namespace tap::motor;
using namespace testing;
using namespace aruwsrc::control::turret;

class TurretSubsystemTest : public Test
{
protected:
    TurretSubsystemTest()
        : pitchMotor(&drivers, MOTOR1, CanBus::CAN_BUS1, false, "pitch"),
          yawMotor(&drivers, MOTOR2, CanBus::CAN_BUS1, false, "yaw"),
          turret(&drivers, &pitchMotor, &yawMotor)
    {
    }

    void SetUp() override
    {
        ON_CALL(pitchMotor, isMotorOnline).WillByDefault(ReturnPointee(&pitchMotorOnline));
        ON_CALL(pitchMotor, getEncoderWrapped).WillByDefault(ReturnPointee(&pitchEncoderWrapped));
        ON_CALL(yawMotor, isMotorOnline).WillByDefault(ReturnPointee(&yawMotorOnline));
        ON_CALL(yawMotor, getEncoderWrapped).WillByDefault(ReturnPointee(&yawEncoderWrapped));
    }

    Drivers drivers;
    NiceMock<DjiMotorMock> pitchMotor;
    NiceMock<DjiMotorMock> yawMotor;
    TurretSubsystem turret;
    bool yawMotorOnline = true;
    bool pitchMotorOnline = true;
    uint16_t pitchEncoderWrapped = 0;
    uint16_t yawEncoderWrapped = 0;
};

TEST_F(TurretSubsystemTest, initialize__initializes_both_motors)
{
    EXPECT_CALL(pitchMotor, initialize);
    EXPECT_CALL(yawMotor, initialize);

    turret.initialize();
}

TEST_F(TurretSubsystemTest, setYawSetpoint__limited_to_min_max_when_limitYaw_false)
{
    // Default expectations so turret assumes motors are good to go and within valid angle range
    std::vector<std::tuple<float, float>> limitedAndInputAnglePairs{
        {YAW_START_ANGLE, YAW_START_ANGLE},
        {YAW_MIN_ANGLE, YAW_MIN_ANGLE - 5},
        {YAW_MAX_ANGLE, YAW_MAX_ANGLE + 5}};

    for (auto [expectedAngle, inputAngle] : limitedAndInputAnglePairs)
    {
        turret.setYawSetpoint(inputAngle);
        EXPECT_NEAR(expectedAngle, turret.getYawSetpoint(), 1E-3);
    }
}

TEST_F(TurretSubsystemTest, setYawSetpoint__not_limited_when_limitYaw_false)
{
    TurretSubsystem turret(&drivers, &pitchMotor, &yawMotor, false);

    std::vector<std::tuple<float, float>> limitedAndInputAnglePairs{
        {YAW_START_ANGLE, YAW_START_ANGLE},
        {tap::algorithms::ContiguousFloat(YAW_MIN_ANGLE - 5, 0, 360).getValue(), YAW_MIN_ANGLE - 5},
        {tap::algorithms::ContiguousFloat(YAW_MAX_ANGLE + 5, 0, 360).getValue(),
         YAW_MAX_ANGLE + 5}};

    for (auto [expectedAngle, inputAngle] : limitedAndInputAnglePairs)
    {
        turret.setYawSetpoint(inputAngle);
        EXPECT_NEAR(expectedAngle, turret.getYawSetpoint(), 1E-3);
    }
}

TEST_F(TurretSubsystemTest, setPitchSetpoint__limited_to_min_max)
{
    std::vector<std::tuple<float, float>> limitedAndInputAnglePairs{
        {PITCH_START_ANGLE, PITCH_START_ANGLE},
        {PITCH_MIN_ANGLE, PITCH_MIN_ANGLE - 5},
        {PITCH_MAX_ANGLE, PITCH_MAX_ANGLE + 5}};

    for (auto [expectedAngle, inputAngle] : limitedAndInputAnglePairs)
    {
        turret.setPitchSetpoint(inputAngle);
        EXPECT_NEAR(expectedAngle, turret.getPitchSetpoint(), 1E-3);
    }
}

TEST_F(TurretSubsystemTest, getCurrentYawValue__returns_default_when_yaw_motor_offline)
{
    // Default expectations so turret assumes motors are good to go and within valid angle range
    pitchMotorOnline = false;
    yawMotorOnline = false;

    turret.refresh();

    EXPECT_NEAR(YAW_START_ANGLE, turret.getCurrentYawValue().getValue(), 1E-3);
}

TEST_F(
    TurretSubsystemTest,
    getCurrentYawValue__returns_values_based_on_enc_position_if_yaw_motor_online)
{
    // Default expectations so turret assumes motors are good to go and within valid angle range
    std::vector<std::tuple<float, int>> angleAndEncoderPairs{
        {90, YAW_START_ENCODER_POSITION},
        {135, YAW_START_ENCODER_POSITION + DjiMotor::ENC_RESOLUTION / 8},
        {180, YAW_START_ENCODER_POSITION + 2 * DjiMotor::ENC_RESOLUTION / 8},
        {225, YAW_START_ENCODER_POSITION + 3 * DjiMotor::ENC_RESOLUTION / 8},
        {270, YAW_START_ENCODER_POSITION + 4 * DjiMotor::ENC_RESOLUTION / 8},
        {315, YAW_START_ENCODER_POSITION + 5 * DjiMotor::ENC_RESOLUTION / 8},
        {0, YAW_START_ENCODER_POSITION + 6 * DjiMotor::ENC_RESOLUTION / 8}};

    for (auto [angle, encoder] : angleAndEncoderPairs)
    {
        yawEncoderWrapped = encoder % DjiMotor::ENC_RESOLUTION;
        turret.refresh();
        EXPECT_NEAR(0.0f, turret.getCurrentYawValue().difference(angle), 1E-3);
    }
}

TEST_F(TurretSubsystemTest, getCurrentPitchValue__returns_default_when_pitch_motor_offline)
{
    // Default expectations so turret assumes motors are good to go and within valid angle range
    pitchMotorOnline = false;

    turret.refresh();

    EXPECT_NEAR(PITCH_START_ANGLE, turret.getCurrentPitchValue().getValue(), 1E-3);
}

TEST_F(
    TurretSubsystemTest,
    getCurrentPitchValue__returns_values_based_on_enc_position_if_pitch_motor_online)
{
    std::vector<std::tuple<float, int>> angleAndEncoderPairs{
        {90, PITCH_START_ENCODER_POSITION},
        {135, PITCH_START_ENCODER_POSITION + DjiMotor::ENC_RESOLUTION / 8},
        {180, PITCH_START_ENCODER_POSITION + 2 * DjiMotor::ENC_RESOLUTION / 8},
        {225, PITCH_START_ENCODER_POSITION + 3 * DjiMotor::ENC_RESOLUTION / 8},
        {270, PITCH_START_ENCODER_POSITION + 4 * DjiMotor::ENC_RESOLUTION / 8},
        {315, PITCH_START_ENCODER_POSITION + 5 * DjiMotor::ENC_RESOLUTION / 8},
        {0, PITCH_START_ENCODER_POSITION + 6 * DjiMotor::ENC_RESOLUTION / 8}};

    for (auto [angle, encoder] : angleAndEncoderPairs)
    {
        pitchEncoderWrapped = encoder % DjiMotor::ENC_RESOLUTION;
        turret.refresh();
        EXPECT_NEAR(0, turret.getCurrentPitchValue().difference(angle), 1E-3);
    }
}

TEST_F(TurretSubsystemTest, onHardwareTestStart__sets_des_out_0)
{
    // Default expectations so turret assumes motors are good to go and within valid angle range
    pitchEncoderWrapped = PITCH_START_ENCODER_POSITION;
    yawEncoderWrapped = YAW_START_ENCODER_POSITION;

    turret.refresh();

    // Make sure desired output isn't 0
    {
        InSequence seq;
        EXPECT_CALL(pitchMotor, setDesiredOutput(Ne(0)));
        EXPECT_CALL(pitchMotor, setDesiredOutput(0));
    }

    {
        InSequence seq;
        EXPECT_CALL(yawMotor, setDesiredOutput(Ne(0)));
        EXPECT_CALL(yawMotor, setDesiredOutput(0));
    }

    turret.setYawMotorOutput(100);
    turret.setPitchMotorOutput(100);

    turret.onHardwareTestStart();
}

TEST_F(TurretSubsystemTest, isOnline__return_true_when_all_motors_online)
{
    EXPECT_TRUE(turret.isOnline());
}

TEST_F(TurretSubsystemTest, isOnline__return_false_when_some_motors_offline)
{
    TurretSubsystem turret(&drivers, &pitchMotor, &yawMotor);

    pitchMotorOnline = false;
    yawMotorOnline = false;
    EXPECT_FALSE(turret.isOnline());

    pitchMotorOnline = true;
    yawMotorOnline = false;
    EXPECT_FALSE(turret.isOnline());

    pitchMotorOnline = false;
    yawMotorOnline = true;
    EXPECT_FALSE(turret.isOnline());
}

TEST_F(TurretSubsystemTest, getPitchAngleFromCenter__return_0_when_motors_offline)
{
    yawMotorOnline = false;
    pitchMotorOnline = false;

    turret.refresh();

    EXPECT_NEAR(0, turret.getPitchAngleFromCenter(), 1E-3);
}

TEST_F(TurretSubsystemTest, getPitchAngleFromCenter__valid_encoder_angles)
{
    std::vector<std::tuple<float, int>> angleAndEncoderPairs{
        {0, PITCH_START_ENCODER_POSITION},
        {45, PITCH_START_ENCODER_POSITION + DjiMotor::ENC_RESOLUTION / 8},
        {90, PITCH_START_ENCODER_POSITION + 2 * DjiMotor::ENC_RESOLUTION / 8},
        {135, PITCH_START_ENCODER_POSITION + 3 * DjiMotor::ENC_RESOLUTION / 8},
        {180, PITCH_START_ENCODER_POSITION + 4 * DjiMotor::ENC_RESOLUTION / 8},
        {-135, PITCH_START_ENCODER_POSITION + 5 * DjiMotor::ENC_RESOLUTION / 8},
        {-90, PITCH_START_ENCODER_POSITION + 6 * DjiMotor::ENC_RESOLUTION / 8}};

    for (auto [angle, encoder] : angleAndEncoderPairs)
    {
        pitchEncoderWrapped = encoder % DjiMotor::ENC_RESOLUTION;
        turret.refresh();
        EXPECT_NEAR(angle, turret.getPitchAngleFromCenter(), 1E-3);
    }
}

TEST_F(TurretSubsystemTest, getYawAngleFromCenter__return_0_when_motors_offline)
{
    yawMotorOnline = false;
    pitchMotorOnline = false;

    turret.refresh();

    EXPECT_NEAR(0, turret.getYawAngleFromCenter(), 1E-3);
}

TEST_F(TurretSubsystemTest, getYawAngleFromCenter)
{
    std::vector<std::tuple<float, int>> angleFromCenterEncoderPairs{
        {0, YAW_START_ENCODER_POSITION},
        {45, YAW_START_ENCODER_POSITION + DjiMotor::ENC_RESOLUTION / 8},
        {90, YAW_START_ENCODER_POSITION + 2 * DjiMotor::ENC_RESOLUTION / 8},
        {135, YAW_START_ENCODER_POSITION + 3 * DjiMotor::ENC_RESOLUTION / 8},
        {180, YAW_START_ENCODER_POSITION + 4 * DjiMotor::ENC_RESOLUTION / 8},
        {-135, YAW_START_ENCODER_POSITION + 5 * DjiMotor::ENC_RESOLUTION / 8},
        {-90, YAW_START_ENCODER_POSITION + 6 * DjiMotor::ENC_RESOLUTION / 8}};

    for (auto [angle, encoder] : angleFromCenterEncoderPairs)
    {
        yawEncoderWrapped = encoder % DjiMotor::ENC_RESOLUTION;
        turret.refresh();
        EXPECT_NEAR(angle, turret.getYawAngleFromCenter(), 1E-3);
    }
}

TEST_F(TurretSubsystemTest, yawLimited__return_matches_constructor_value)
{
    TurretSubsystem t1(&drivers, &pitchMotor, &yawMotor);
    TurretSubsystem t2(&drivers, &pitchMotor, &yawMotor, false);

    EXPECT_TRUE(t1.yawLimited());
    EXPECT_FALSE(t2.yawLimited());
}

TEST_F(TurretSubsystemTest, setPitchMotorOutput__desired_output_0_when_turret_offline)
{
    pitchMotorOnline = false;

    EXPECT_CALL(pitchMotor, setDesiredOutput).Times(0);

    turret.setPitchMotorOutput(1000);
}

TEST_F(
    TurretSubsystemTest,
    setPitchMotorOutput__desired_output_identical_to_input_when_turret_online_and_enc_within_bounds)
{
    pitchEncoderWrapped = PITCH_START_ENCODER_POSITION;

    InSequence seq;
    EXPECT_CALL(pitchMotor, setDesiredOutput(1000));
    EXPECT_CALL(pitchMotor, setDesiredOutput(-1000));

    turret.setPitchMotorOutput(1000);

    turret.setPitchMotorOutput(-1000);
}

TEST_F(
    TurretSubsystemTest,
    setPitchMotorOutput__desired_output_not_limited_if_equal_to_min_max_bound)
{
    uint16_t minEncoderValue =
        tap::algorithms::ContiguousFloat(
            PITCH_START_ENCODER_POSITION +
                +(PITCH_MIN_ANGLE - PITCH_START_ANGLE) * DjiMotor::ENC_RESOLUTION / 360.0f + 1,
            0,
            DjiMotor::ENC_RESOLUTION)
            .getValue();

    uint16_t maxEncoderValue =
        tap::algorithms::ContiguousFloat(
            PITCH_START_ENCODER_POSITION +
                +(PITCH_MAX_ANGLE - PITCH_START_ANGLE) * DjiMotor::ENC_RESOLUTION / 360.0f - 1,
            0,
            DjiMotor::ENC_RESOLUTION)
            .getValue();

    InSequence seq;
    EXPECT_CALL(pitchMotor, setDesiredOutput(-1000));
    EXPECT_CALL(pitchMotor, setDesiredOutput(1000));

    // desired output negative, equal to min
    pitchEncoderWrapped = minEncoderValue;
    turret.refresh();
    turret.setPitchMotorOutput(-1000);

    // desired output position, equal to max
    pitchEncoderWrapped = maxEncoderValue;
    turret.refresh();
    turret.setPitchMotorOutput(1000);
}

TEST_F(
    TurretSubsystemTest,
    setPitchMotorOutput__desired_output_0_if_out_of_bounds_and_input_des_output_wrong_way)
{
    uint16_t lessThanMinEncoderValue =
        tap::algorithms::ContiguousFloat(
            PITCH_START_ENCODER_POSITION +
                +(PITCH_MIN_ANGLE - PITCH_START_ANGLE - 10) * DjiMotor::ENC_RESOLUTION / 360.0f,
            0,
            DjiMotor::ENC_RESOLUTION)
            .getValue();

    uint16_t greaterThanMaxEncoderValue =
        tap::algorithms::ContiguousFloat(
            PITCH_START_ENCODER_POSITION +
                +(PITCH_MAX_ANGLE - PITCH_START_ANGLE + 10) * DjiMotor::ENC_RESOLUTION / 360.0f,
            0,
            DjiMotor::ENC_RESOLUTION)
            .getValue();

    InSequence seq;
    EXPECT_CALL(pitchMotor, setDesiredOutput(0));
    EXPECT_CALL(pitchMotor, setDesiredOutput(0));

    // desired output negative, angle less than min
    pitchEncoderWrapped = lessThanMinEncoderValue;
    turret.refresh();
    turret.setPitchMotorOutput(-1000);

    // desired output position, angle greater than max
    pitchEncoderWrapped = greaterThanMaxEncoderValue;
    turret.refresh();
    turret.setPitchMotorOutput(1000);
}

TEST_F(TurretSubsystemTest, setYawMotorOutput__desired_output_0_when_turret_offline)
{
    yawMotorOnline = false;

    EXPECT_CALL(yawMotor, setDesiredOutput).Times(0);

    turret.setYawMotorOutput(1000);
}

TEST_F(
    TurretSubsystemTest,
    setYawMotorOutput__desired_output_identical_to_input_when_turret_online_and_enc_within_bounds)
{
    yawEncoderWrapped = YAW_START_ENCODER_POSITION;

    turret.refresh();

    InSequence seq;
    EXPECT_CALL(yawMotor, setDesiredOutput(1000));
    EXPECT_CALL(yawMotor, setDesiredOutput(-1000));

    turret.setYawMotorOutput(1000);

    turret.setYawMotorOutput(-1000);
}

TEST_F(TurretSubsystemTest, setYawMotorOutput__desired_output_not_limited_if_equal_to_min_max_bound)
{
    uint16_t minEncoderValue =
        tap::algorithms::ContiguousFloat(
            YAW_START_ENCODER_POSITION +
                +(YAW_MIN_ANGLE - YAW_START_ANGLE) * DjiMotor::ENC_RESOLUTION / 360.0f + 1,
            0,
            DjiMotor::ENC_RESOLUTION)
            .getValue();

    uint16_t maxEncoderValue =
        tap::algorithms::ContiguousFloat(
            YAW_START_ENCODER_POSITION +
                +(YAW_MAX_ANGLE - YAW_START_ANGLE) * DjiMotor::ENC_RESOLUTION / 360.0f - 1,
            0,
            DjiMotor::ENC_RESOLUTION)
            .getValue();

    InSequence seq;
    EXPECT_CALL(yawMotor, setDesiredOutput(-1000));
    EXPECT_CALL(yawMotor, setDesiredOutput(1000));

    // desired output negative, angle equal to min
    yawEncoderWrapped = minEncoderValue;
    turret.refresh();
    turret.setYawMotorOutput(-1000);

    // desired output position, angle equal to max
    yawEncoderWrapped = maxEncoderValue;
    turret.refresh();
    turret.setYawMotorOutput(1000);
}

TEST_F(
    TurretSubsystemTest,
    setYawMotorOutput__desired_output_0_if_out_of_bounds_and_input_des_output_wrong_way)
{
    uint16_t lessThanMinEncoderValue =
        tap::algorithms::ContiguousFloat(
            YAW_START_ENCODER_POSITION +
                +(YAW_MIN_ANGLE - 6 - YAW_START_ANGLE) * DjiMotor::ENC_RESOLUTION / 360.0f,
            0,
            DjiMotor::ENC_RESOLUTION)
            .getValue();

    uint16_t greaterThanMaxEncoderValue =
        tap::algorithms::ContiguousFloat(
            YAW_START_ENCODER_POSITION +
                +(YAW_MAX_ANGLE + 10 - YAW_START_ANGLE) * DjiMotor::ENC_RESOLUTION / 360.0f,
            0,
            DjiMotor::ENC_RESOLUTION)
            .getValue();

    InSequence seq;
    EXPECT_CALL(yawMotor, setDesiredOutput(0));
    EXPECT_CALL(yawMotor, setDesiredOutput(0));

    // desired output negative, angle less than min
    yawEncoderWrapped = lessThanMinEncoderValue;
    turret.refresh();
    turret.setYawMotorOutput(-1000);

    // desired output position, angle greater than max
    yawEncoderWrapped = greaterThanMaxEncoderValue;
    turret.refresh();
    turret.setYawMotorOutput(1000);
}

TEST_F(TurretSubsystemTest, turret_actual_angles_update_when_subsystem_in_scheduler)
{
    tap::control::CommandScheduler scheduler(&drivers, true);

    EXPECT_CALL(yawMotor, getEncoderWrapped).WillOnce(Return(100));
    EXPECT_CALL(pitchMotor, getEncoderWrapped).WillOnce(Return(100));

    scheduler.registerSubsystem(&turret);
    scheduler.run();
}

TEST_F(TurretSubsystemTest, refresh_sets_actual_angle_back_to_start_when_offline)
{
    // Initially turret online
    yawEncoderWrapped = YAW_START_ENCODER_POSITION + 1000;
    pitchEncoderWrapped = PITCH_START_ENCODER_POSITION + 1000;

    turret.refresh();

    EXPECT_NE(YAW_START_ANGLE, turret.getCurrentYawValue().getValue());
    EXPECT_NE(PITCH_START_ANGLE, turret.getCurrentPitchValue().getValue());

    // Now turret offline
    yawMotorOnline = false;
    pitchMotorOnline = false;

    turret.refresh();

    EXPECT_NEAR(YAW_START_ANGLE, turret.getCurrentYawValue().getValue(), 1E-3);
    EXPECT_NEAR(PITCH_START_ANGLE, turret.getCurrentPitchValue().getValue(), 1E-3);
}
