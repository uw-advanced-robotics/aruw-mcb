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
          turretMotor(&motor, TURRET_MOTOR_CONFIG)
    {
    }

    void SetUp() override
    {
        ON_CALL(motor, isMotorOnline).WillByDefault(ReturnPointee(&motorOnline));
        ON_CALL(motor, getEncoderWrapped).WillByDefault(ReturnPointee(&encoderWrapped));
        ON_CALL(motor, getEncoderUnwrapped).WillByDefault(ReturnPointee(&encoderUnwrapped));
    }

    void setEncoder(int64_t encoderUnwrapped)
    {
        this->encoderUnwrapped = encoderUnwrapped;
        encoderWrapped = encoderUnwrapped % DjiMotor::ENC_RESOLUTION;
    }

    Drivers drivers;
    NiceMock<DjiMotorMock> motor;
    TurretMotor turretMotor;
    bool motorOnline = true;

private:
    uint16_t encoderWrapped = 0;
    int64_t encoderUnwrapped = 0;
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
        EXPECT_NEAR(expectedAngle, turretMotor.getChassisFrameSetpoint(), 1E-3);
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
        turretMotor.setChassisFrameSetpoint(expectedAngle);
        EXPECT_NEAR(expectedAngle, turretMotor.getChassisFrameSetpoint(), 1E-3);
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
        setEncoder(encoder);
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
        setEncoder(encoder);
        turretMotor.updateMotorAngle();
        EXPECT_NEAR(angle, turretMotor.getAngleFromCenter(), 1E-3);
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
    setEncoder(TURRET_MOTOR_CONFIG.startEncoderValue);

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
    setEncoder(minEncoderValue);
    turretMotor.setMotorOutput(-1000);

    // desired output position, equal to max
    setEncoder(maxEncoderValue);
    turretMotor.updateMotorAngle();
    turretMotor.setMotorOutput(1000);
}

TEST_F(TurretMotorTest, updateMotorAngle_sets_actual_angle_back_to_start_when_offline)
{
    // Initially turret online
    setEncoder(TURRET_MOTOR_CONFIG.startEncoderValue + 1000);

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
        EXPECT_NEAR(error, tm.getValidMinError(setpoint, measurement), 1E-3);
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
        EXPECT_NEAR(error, tm.getValidMinError(setpoint, measurement), 1E-3);
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

    tm.setChassisFrameSetpoint(-M_PI_2 - M_PI_4 / 2.0f);
    EXPECT_NEAR(-M_PI_2, tm.getChassisFrameSetpoint(), 1E-3);

    tm.setChassisFrameSetpoint(M_PI + M_PI_4 / 2.0f);
    EXPECT_NEAR(M_PI, tm.getChassisFrameSetpoint(), 1E-3);
}

static int64_t getEncoderUnwrapped(const TurretMotorConfig &motorConfig, float angle)
{
    return static_cast<int64_t>(motorConfig.startEncoderValue) +
           static_cast<int64_t>(DjiMotor::ENC_RESOLUTION) * (angle - motorConfig.startAngle) /
               M_TWOPI;
}

TEST_F(TurretMotorTest, getValidChassisMeasurementError_various_setpoints)
{
    TurretMotorConfig mc = {
        .startAngle = 0,
        .startEncoderValue = 0,
        .minAngle = -M_TWOPI,
        .maxAngle = M_TWOPI,
        .limitMotorAngles = true,
    };
    TurretMotor tm(&motor, mc);

    motorOnline = true;

    std::vector<std::tuple<float, float, float>> errorMeasurementsToTest = {
        {-M_TWOPI, -M_TWOPI, 0},
        {-M_TWOPI, 0, M_TWOPI},
        {-M_TWOPI, M_TWOPI, 2 * M_TWOPI},
        {M_TWOPI, -M_TWOPI, -2 * M_TWOPI},
    };

    setEncoder(getEncoderUnwrapped(mc, mc.startAngle));
    tm.updateMotorAngle();

    for (auto [measured, setpoint, expectedErr] : errorMeasurementsToTest)
    {
        setEncoder(getEncoderUnwrapped(mc, measured));
        tm.updateMotorAngle();

        EXPECT_NEAR(measured, tm.getChassisFrameUnwrappedMeasuredAngle(), 1E-3);

        tm.setChassisFrameSetpoint(setpoint);

        EXPECT_NEAR(setpoint, tm.getChassisFrameSetpoint(), 1E-3);

        EXPECT_NEAR(expectedErr, tm.getValidChassisMeasurementError(), 1E-3);
    }
}

TEST(TurretMotor, getClosestNonNormalizedSetpointToMeasurement)
{
    std::vector<std::tuple<float, float, float>> valuesToTest = {
        {0, 0, 0},
        {-M_TWOPI, 0, -M_TWOPI},
        {M_TWOPI, 0, M_TWOPI},
        {-M_TWOPI, 0.1, -M_TWOPI + 0.1},
        {-M_PI, M_PI, -M_PI},
        {M_PI_2, 0, 0},
        {M_PI, 0, 0},
        {M_TWOPI + M_PI_2, -M_TWOPI, M_TWOPI},
        {M_TWOPI + M_PI_2, -M_TWOPI - M_PI, M_TWOPI + M_PI},
    };

    for (auto &[measurement, setpoint, nonNormalizedSetpoint] : valuesToTest)
    {
        EXPECT_NEAR(
            nonNormalizedSetpoint,
            TurretMotor::getClosestNonNormalizedSetpointToMeasurement(measurement, setpoint),
            1E-3);
    }
}

TEST_F(TurretMotorTest, getSetpointWithinTurretRange)
{
    TurretMotorConfig mc = {
        .startAngle = 0,
        .startEncoderValue = 0,
        .minAngle = -M_TWOPI,
        .maxAngle = M_TWOPI,
        .limitMotorAngles = true,
    };
    TurretMotor tm(&motor, mc);

    std::vector<std::tuple<float, float>> valuesToTest = {
        {0, 0},
        {-M_TWOPI, -M_TWOPI},
        {M_TWOPI, M_TWOPI},
        {M_PI, M_PI},
        {-M_PI, -M_PI},
        {-M_TWOPI - 0.1, -0.1},
        {M_TWOPI + 0.1, 0.1},
        {4 * M_TWOPI + 0.1, 0.1},
    };

    for (auto [setpoint, expected] : valuesToTest)
    {
        EXPECT_NEAR(expected, tm.getSetpointWithinTurretRange(setpoint), 1E-3);
    }
}

struct UnwrapTargetAngleTestValues
{
    float targetFrameMeasurement;
    float targetAngle;
    float expectedUnwrappedTargetAngle;  // If unwrapping happens, this is the unwrapped value. If
                                         // unwrapping doesn't happen in a particular test,
                                         // targetAngle should not change.
};

class UnwrapTargetAngleTest
    : public TurretMotorTest,
      public WithParamInterface<std::tuple<UnwrapTargetAngleTestValues, TurretMotorConfig>>
{
protected:
    UnwrapTargetAngleTest() : tm(&motor, std::get<1>(GetParam())), turretController(tm) {}

    void SetUp() override
    {
        TurretMotorTest::SetUp();

        ON_CALL(turretController, getMeasurement)
            .WillByDefault(Return(std::get<0>(GetParam()).targetFrameMeasurement));
    }

    void runUnwrapTargetAngleTest()
    {
        float unwrappedTargetAngle = std::get<0>(GetParam()).targetAngle;
        unwrappedTargetAngle = tm.unwrapTargetAngle(unwrappedTargetAngle);

        if (std::get<1>(GetParam()).limitMotorAngles && tm.getTurretController() != nullptr)
        {
            float controllerFrameExpectedTarget =
                std::get<0>(GetParam()).expectedUnwrappedTargetAngle;

            // using getSetpointWithinTurretRange since that has been validated to work by another
            // test
            float chassisFrameExpectedTarget =
                turretController.convertControllerAngleToChassisFrame(
                    controllerFrameExpectedTarget);

            controllerFrameExpectedTarget = turretController.convertChassisAngleToControllerFrame(
                tm.getSetpointWithinTurretRange(chassisFrameExpectedTarget));

            EXPECT_NEAR(controllerFrameExpectedTarget, unwrappedTargetAngle, 1e-5f);
        }
        else
        {
            // targetAngle should not change
            EXPECT_EQ(std::get<0>(GetParam()).targetAngle, unwrappedTargetAngle);
        }
    }

    TurretMotor tm;
    NiceMock<aruwsrc::mock::TurretControllerInterfaceMock> turretController;
};

TEST_P(UnwrapTargetAngleTest, unwrapped_angle_correct_no_turret_controller)
{
    runUnwrapTargetAngleTest();
}

TEST_P(UnwrapTargetAngleTest, unwrapped_angle_correct_chassis_frame_controller)
{
    ON_CALL(turretController, convertChassisAngleToControllerFrame).WillByDefault([](float value) {
        return value;
    });
    ON_CALL(turretController, convertControllerAngleToChassisFrame).WillByDefault([](float value) {
        return value;
    });

    tm.attachTurretController(&turretController);

    runUnwrapTargetAngleTest();
}

TEST_P(UnwrapTargetAngleTest, unwrapped_angle_correct_rotated_frame_controller)
{
    ON_CALL(turretController, convertChassisAngleToControllerFrame).WillByDefault([](float value) {
        return value + M_PI;
    });
    ON_CALL(turretController, convertControllerAngleToChassisFrame).WillByDefault([](float value) {
        return value - M_PI;
    });

    tm.attachTurretController(&turretController);

    float chassisFrameExpectedTarget = turretController.convertControllerAngleToChassisFrame(
        std::get<0>(GetParam()).expectedUnwrappedTargetAngle);
    chassisFrameExpectedTarget = turretController.convertChassisAngleToControllerFrame(
        turretMotor.getSetpointWithinTurretRange(chassisFrameExpectedTarget));

    runUnwrapTargetAngleTest();
}

std::vector<UnwrapTargetAngleTestValues> unwrapTargetAnglesValuesToTest = {
    {
        .targetFrameMeasurement = 0,
        .targetAngle = 0,
        .expectedUnwrappedTargetAngle = 0,
    },
    {
        .targetFrameMeasurement = 0,
        .targetAngle = M_PI_2,
        .expectedUnwrappedTargetAngle = M_PI_2,
    },
    {
        .targetFrameMeasurement = 0,
        .targetAngle = -M_PI_2,
        .expectedUnwrappedTargetAngle = -M_PI_2,
    },
    {
        .targetFrameMeasurement = 0,
        .targetAngle = -M_TWOPI,
        .expectedUnwrappedTargetAngle = 0,
    },
    {
        .targetFrameMeasurement = 0,
        .targetAngle = M_TWOPI,
        .expectedUnwrappedTargetAngle = 0,
    },
    {
        .targetFrameMeasurement = M_PI,
        .targetAngle = -M_PI,
        .expectedUnwrappedTargetAngle = M_PI,
    },
    {
        .targetFrameMeasurement = M_PI,
        .targetAngle = -M_PI - M_PI_4,
        .expectedUnwrappedTargetAngle = M_PI - M_PI_4,
    },
    {
        .targetFrameMeasurement = -M_PI,
        .targetAngle = M_PI + M_PI_4,
        .expectedUnwrappedTargetAngle = -M_PI + M_PI_4,
    },
};

std::vector<TurretMotorConfig> motorConfigValuesToTest = {
    {
        .startAngle = 0,
        .startEncoderValue = 0,
        .minAngle = 0,
        .maxAngle = 0,
        .limitMotorAngles = false,
    },
    {
        .startAngle = 0,
        .startEncoderValue = 0,
        .minAngle = -M_PI,
        .maxAngle = M_PI,
        .limitMotorAngles = true,
    },
    {
        .startAngle = 0,
        .startEncoderValue = 0,
        .minAngle = -M_TWOPI,
        .maxAngle = M_TWOPI,
        .limitMotorAngles = true,
    },
};

INSTANTIATE_TEST_SUITE_P(
    TurretMotorTest,
    UnwrapTargetAngleTest,
    Combine(ValuesIn(unwrapTargetAnglesValuesToTest), ValuesIn(motorConfigValuesToTest)));
