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
        turretMotor(&motor, TURRET_MOTOR_CONFIG)
    {
    }

    void SetUp() override
    {
        ON_CALL(motor, isMotorOnline).WillByDefault(ReturnPointee(&motorOnline));
        ON_CALL(motor, getEncoderWrapped).WillByDefault(ReturnPointee(&encoderWrapped));
    }

    Drivers drivers;
    NiceMock<DjiMotorMock> motor;
    TurretMotor turretMotor;
    bool motorOnline = true;
    uint16_t encoderWrapped = 0;
};

TEST_F(TurretMotorTest, isOnline_reflective_of_motor_online) {
    motorOnline = true;
    EXPECT_TRUE(turretMotor.isOnline());
    motorOnline = false;
    EXPECT_FALSE(turretMotor.isOnline());
}

TEST_F(TurretMotorTest, initialize__initializes_both_motors)
{
    EXPECT_CALL(motor, initialize);

    turretMotor.initialize();
}

TEST_F(TurretMotorTest, setChassisFrameSetpoint__limited_to_min_max_when_limitYaw_false)
{
    // Default expectations so turret assumes motors are good to go and within valid angle range
    std::vector<std::tuple<float, float>> limitedAndInputAnglePairs{
        {TURRET_MOTOR_CONFIG.startAngle, TURRET_MOTOR_CONFIG.startAngle},
        {TURRET_MOTOR_CONFIG.minAngle, TURRET_MOTOR_CONFIG.maxAngle - 5},
        {TURRET_MOTOR_CONFIG.maxAngle, TURRET_MOTOR_CONFIG.maxAngle + 5}};

    for (auto [expectedAngle, inputAngle] : limitedAndInputAnglePairs)
    {
        turretMotor.setChassisFrameSetpoint(inputAngle);
        EXPECT_NEAR(expectedAngle, turretMotor.getChassisFrameSetpoint().getValue(), 1E-3);
    }
}

TEST_F(TurretMotorTest, setChassisFrameSetpoint__not_limited_when_limitYaw_false)
{
    TurretMotorConfig motorConfig = TURRET_MOTOR_CONFIG;
    motorConfig.limitMotorAngles = false;

    TurretMotor turret(&motor, motorConfig);

    std::vector<std::tuple<float, float>> limitedAndInputAnglePairs{
        {motorConfig.startAngle, motorConfig.startAngle},
        {tap::algorithms::ContiguousFloat(motorConfig.minAngle - 5, 0, M_TWOPI).getValue(), motorConfig.minAngle - 5},
        {tap::algorithms::ContiguousFloat(motorConfig.maxAngle + 5, 0, M_TWOPI).getValue(),
         motorConfig.maxAngle + 5}};

    for (auto [expectedAngle, inputAngle] : limitedAndInputAnglePairs)
    {
        turretMotor.setChassisFrameSetpoint(inputAngle);
        EXPECT_NEAR(expectedAngle, turretMotor.getChassisFrameSetpoint().getValue(), 1E-3);
    }
}

TEST_F(TurretMotorTest, getMeasuredYawValue__returns_default_when_yaw_motor_offline)
{
    // Default expectations so turret assumes motors are good to go and within valid angle range
    motorOnline = false;

    turretMotor.updateMotorAngle();

    EXPECT_NEAR(TURRET_MOTOR_CONFIG.startAngle, turretMotor.getChassisFrameMeasuredAngle().getValue(), 1E-3);
}

// TEST_F(
//     TurretMotorTest,
//     getMeasuredYawValue__returns_values_based_on_enc_position_if_yaw_motor_online)
// {
//     // Default expectations so turret assumes motors are good to go and within valid angle range
//     std::vector<std::tuple<float, int>> angleAndEncoderPairs{
//         {90, YAW_START_ENCODER_POSITION},
//         {135, YAW_START_ENCODER_POSITION + DjiMotor::ENC_RESOLUTION / 8},
//         {180, YAW_START_ENCODER_POSITION + 2 * DjiMotor::ENC_RESOLUTION / 8},
//         {225, YAW_START_ENCODER_POSITION + 3 * DjiMotor::ENC_RESOLUTION / 8},
//         {270, YAW_START_ENCODER_POSITION + 4 * DjiMotor::ENC_RESOLUTION / 8},
//         {315, YAW_START_ENCODER_POSITION + 5 * DjiMotor::ENC_RESOLUTION / 8},
//         {0, YAW_START_ENCODER_POSITION + 6 * DjiMotor::ENC_RESOLUTION / 8}};

//     for (auto [angle, encoder] : angleAndEncoderPairs)
//     {
//         yawEncoderWrapped = encoder % DjiMotor::ENC_RESOLUTION;
//         turretMotor.refresh();
//         EXPECT_NEAR(0.0f, turretMotor.getMeasuredYawValue().difference(angle), 1E-3);
//     }
// }




// TEST_F(TurretMotorTest, getPitchAngleFromCenter__return_0_when_motors_offline)
// {
//     yawMotorOnline = false;
//     pitchMotorOnline = false;

//     turretMotor.refresh();

//     EXPECT_NEAR(0, turretMotor.getPitchAngleFromCenter(), 1E-3);
// }

// TEST_F(TurretMotorTest, getPitchAngleFromCenter__valid_encoder_angles)
// {
//     std::vector<std::tuple<float, int>> angleAndEncoderPairs{
//         {0, PITCH_START_ENCODER_POSITION},
//         {45, PITCH_START_ENCODER_POSITION + DjiMotor::ENC_RESOLUTION / 8},
//         {90, PITCH_START_ENCODER_POSITION + 2 * DjiMotor::ENC_RESOLUTION / 8},
//         {135, PITCH_START_ENCODER_POSITION + 3 * DjiMotor::ENC_RESOLUTION / 8},
//         {180, PITCH_START_ENCODER_POSITION + 4 * DjiMotor::ENC_RESOLUTION / 8},
//         {-135, PITCH_START_ENCODER_POSITION + 5 * DjiMotor::ENC_RESOLUTION / 8},
//         {-90, PITCH_START_ENCODER_POSITION + 6 * DjiMotor::ENC_RESOLUTION / 8}};

//     for (auto [angle, encoder] : angleAndEncoderPairs)
//     {
//         pitchEncoderWrapped = encoder % DjiMotor::ENC_RESOLUTION;
//         turretMotor.refresh();
//         EXPECT_NEAR(angle, turretMotor.getPitchAngleFromCenter(), 1E-3);
//     }
// }


// TEST_F(TurretMotorTest, setPitchMotorOutput__desired_output_0_when_turret_offline)
// {
//     pitchMotorOnline = false;

//     EXPECT_CALL(pitchMotor, setDesiredOutput).Times(0);

//     turretMotor.setPitchMotorOutput(1000);
// }

// TEST_F(
//     TurretMotorTest,
//     setPitchMotorOutput__desired_output_identical_to_input_when_turret_online_and_enc_within_bounds)
// {
//     pitchEncoderWrapped = PITCH_START_ENCODER_POSITION;

//     InSequence seq;
//     EXPECT_CALL(pitchMotor, setDesiredOutput(1000));
//     EXPECT_CALL(pitchMotor, setDesiredOutput(-1000));

//     turretMotor.setPitchMotorOutput(1000);

//     turretMotor.setPitchMotorOutput(-1000);
// }

// TEST_F(
//     TurretMotorTest,
//     setPitchMotorOutput__desired_output_not_limited_if_equal_to_min_max_bound)
// {
//     uint16_t minEncoderValue =
//         tap::algorithms::ContiguousFloat(
//             PITCH_START_ENCODER_POSITION +
//                 +(PITCH_MIN_ANGLE - PITCH_START_ANGLE) * DjiMotor::ENC_RESOLUTION / M_TWOPI.0f + 1,
//             0,
//             DjiMotor::ENC_RESOLUTION)
//             .getValue();

//     uint16_t maxEncoderValue =
//         tap::algorithms::ContiguousFloat(
//             PITCH_START_ENCODER_POSITION +
//                 +(PITCH_MAX_ANGLE - PITCH_START_ANGLE) * DjiMotor::ENC_RESOLUTION / M_TWOPI.0f - 1,
//             0,
//             DjiMotor::ENC_RESOLUTION)
//             .getValue();

//     InSequence seq;
//     EXPECT_CALL(pitchMotor, setDesiredOutput(-1000));
//     EXPECT_CALL(pitchMotor, setDesiredOutput(1000));

//     // desired output negative, equal to min
//     pitchEncoderWrapped = minEncoderValue;
//     turretMotor.refresh();
//     turretMotor.setPitchMotorOutput(-1000);

//     // desired output position, equal to max
//     pitchEncoderWrapped = maxEncoderValue;
//     turretMotor.refresh();
//     turretMotor.setPitchMotorOutput(1000);
// }

// TEST_F(
//     TurretMotorTest,
//     setPitchMotorOutput__desired_output_0_if_out_of_bounds_and_input_des_output_wrong_way)
// {
//     uint16_t lessThanMinEncoderValue =
//         tap::algorithms::ContiguousFloat(
//             PITCH_START_ENCODER_POSITION +
//                 +(PITCH_MIN_ANGLE - PITCH_START_ANGLE - 10) * DjiMotor::ENC_RESOLUTION / M_TWOPI.0f,
//             0,
//             DjiMotor::ENC_RESOLUTION)
//             .getValue();

//     uint16_t greaterThanMaxEncoderValue =
//         tap::algorithms::ContiguousFloat(
//             PITCH_START_ENCODER_POSITION +
//                 +(PITCH_MAX_ANGLE - PITCH_START_ANGLE + 10) * DjiMotor::ENC_RESOLUTION / M_TWOPI.0f,
//             0,
//             DjiMotor::ENC_RESOLUTION)
//             .getValue();

//     InSequence seq;
//     EXPECT_CALL(pitchMotor, setDesiredOutput(0));
//     EXPECT_CALL(pitchMotor, setDesiredOutput(0));

//     // desired output negative, angle less than min
//     pitchEncoderWrapped = lessThanMinEncoderValue;
//     turretMotor.refresh();
//     turretMotor.setPitchMotorOutput(-1000);

//     // desired output position, angle greater than max
//     pitchEncoderWrapped = greaterThanMaxEncoderValue;
//     turretMotor.refresh();
//     turretMotor.setPitchMotorOutput(1000);
// }

// TEST_F(TurretMotorTest, setYawMotorOutput__desired_output_0_when_turret_offline)
// {
//     yawMotorOnline = false;

//     EXPECT_CALL(yawMotor, setDesiredOutput).Times(0);

//     turretMotor.setYawMotorOutput(1000);
// }

// TEST_F(
//     TurretMotorTest,
//     setYawMotorOutput__desired_output_identical_to_input_when_turret_online_and_enc_within_bounds)
// {
//     yawEncoderWrapped = YAW_START_ENCODER_POSITION;

//     turretMotor.refresh();

//     InSequence seq;
//     EXPECT_CALL(yawMotor, setDesiredOutput(1000));
//     EXPECT_CALL(yawMotor, setDesiredOutput(-1000));

//     turretMotor.setYawMotorOutput(1000);

//     turretMotor.setYawMotorOutput(-1000);
// }

// TEST_F(TurretMotorTest, setYawMotorOutput__desired_output_not_limited_if_equal_to_min_max_bound)
// {
//     uint16_t minEncoderValue =
//         tap::algorithms::ContiguousFloat(
//             YAW_START_ENCODER_POSITION +
//                 +(YAW_MIN_ANGLE - YAW_START_ANGLE) * DjiMotor::ENC_RESOLUTION / M_TWOPI.0f + 1,
//             0,
//             DjiMotor::ENC_RESOLUTION)
//             .getValue();

//     uint16_t maxEncoderValue =
//         tap::algorithms::ContiguousFloat(
//             YAW_START_ENCODER_POSITION +
//                 +(YAW_MAX_ANGLE - YAW_START_ANGLE) * DjiMotor::ENC_RESOLUTION / M_TWOPI.0f - 1,
//             0,
//             DjiMotor::ENC_RESOLUTION)
//             .getValue();

//     InSequence seq;
//     EXPECT_CALL(yawMotor, setDesiredOutput(-1000));
//     EXPECT_CALL(yawMotor, setDesiredOutput(1000));

//     // desired output negative, angle equal to min
//     yawEncoderWrapped = minEncoderValue;
//     turretMotor.refresh();
//     turretMotor.setYawMotorOutput(-1000);

//     // desired output position, angle equal to max
//     yawEncoderWrapped = maxEncoderValue;
//     turretMotor.refresh();
//     turretMotor.setYawMotorOutput(1000);
// }

// TEST_F(
//     TurretMotorTest,
//     setYawMotorOutput__desired_output_0_if_out_of_bounds_and_input_des_output_wrong_way)
// {
//     uint16_t lessThanMinEncoderValue =
//         tap::algorithms::ContiguousFloat(
//             YAW_START_ENCODER_POSITION +
//                 +(YAW_MIN_ANGLE - 6 - YAW_START_ANGLE) * DjiMotor::ENC_RESOLUTION / M_TWOPI.0f,
//             0,
//             DjiMotor::ENC_RESOLUTION)
//             .getValue();

//     uint16_t greaterThanMaxEncoderValue =
//         tap::algorithms::ContiguousFloat(
//             YAW_START_ENCODER_POSITION +
//                 +(YAW_MAX_ANGLE + 10 - YAW_START_ANGLE) * DjiMotor::ENC_RESOLUTION / M_TWOPI.0f,
//             0,
//             DjiMotor::ENC_RESOLUTION)
//             .getValue();

//     InSequence seq;
//     EXPECT_CALL(yawMotor, setDesiredOutput(0));
//     EXPECT_CALL(yawMotor, setDesiredOutput(0));

//     // desired output negative, angle less than min
//     yawEncoderWrapped = lessThanMinEncoderValue;
//     turretMotor.refresh();
//     turretMotor.setYawMotorOutput(-1000);

//     // desired output position, angle greater than max
//     yawEncoderWrapped = greaterThanMaxEncoderValue;
//     turretMotor.refresh();
//     turretMotor.setYawMotorOutput(1000);
// }

// TEST_F(TurretMotorTest, turret_actual_angles_update_when_subsystem_in_scheduler)
// {
//     tap::control::CommandScheduler scheduler(&drivers, true);

//     EXPECT_CALL(yawMotor, getEncoderWrapped).WillOnce(Return(100));
//     EXPECT_CALL(pitchMotor, getEncoderWrapped).WillOnce(Return(100));

//     scheduler.registerSubsystem(&turret);
//     scheduler.run();
// }

// TEST_F(TurretMotorTest, refresh_sets_actual_angle_back_to_start_when_offline)
// {
//     // Initially turret online
//     yawEncoderWrapped = YAW_START_ENCODER_POSITION + 1000;
//     pitchEncoderWrapped = PITCH_START_ENCODER_POSITION + 1000;

//     turretMotor.refresh();

//     EXPECT_NE(YAW_START_ANGLE, turretMotor.getMeasuredYawValue().getValue());
//     EXPECT_NE(PITCH_START_ANGLE, turretMotor.getMeasuredPitchValue().getValue());

//     // Now turret offline
//     yawMotorOnline = false;
//     pitchMotorOnline = false;

//     turretMotor.refresh();

//     EXPECT_NEAR(YAW_START_ANGLE, turretMotor.getMeasuredYawValue().getValue(), 1E-3);
//     EXPECT_NEAR(PITCH_START_ANGLE, turretMotor.getMeasuredPitchValue().getValue(), 1E-3);
// }
