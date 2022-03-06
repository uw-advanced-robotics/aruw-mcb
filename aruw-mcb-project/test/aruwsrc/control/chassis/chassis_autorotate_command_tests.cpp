/*
 * Copyright (c) 2020-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "aruwsrc/control/chassis/chassis_autorotate_command.hpp"
#include "aruwsrc/drivers.hpp"
#include "aruwsrc/mock/chassis_subsystem_mock.hpp"
#include "aruwsrc/mock/turret_subsystem_mock.hpp"

using namespace aruwsrc::mock;
using namespace aruwsrc::chassis;
using namespace testing;
using namespace tap::algorithms;
using namespace aruwsrc::control::turret;

static constexpr float TEST_WHEEL_SPEED = MIN_WHEEL_SPEED_SINGLE_MOTOR;

#define DEFAULT_SETUP_TEST(chassisSymmetry)           \
    aruwsrc::Drivers drivers;                         \
    NiceMock<ChassisSubsystemMock> chassis(&drivers); \
    NiceMock<TurretSubsystemMock> turret(&drivers);   \
    ChassisAutorotateCommand cac(&drivers, &chassis, &turret, chassisSymmetry);

#define SET_USER_INPUT(drivers, x, y, r)                                                  \
    ON_CALL(drivers.controlOperatorInterface, getChassisXInput).WillByDefault(Return(x)); \
    ON_CALL(drivers.controlOperatorInterface, getChassisYInput).WillByDefault(Return(y)); \
    ON_CALL(drivers.controlOperatorInterface, getChassisRInput).WillByDefault(Return(r));

#define SET_DEFAULT_REF_SERIAL_BEHAVIOR(drivers)                                        \
    tap::communication::serial::RefSerialData::Rx::RobotData robotData;                 \
    ON_CALL(drivers.refSerial, getRefSerialReceivingData).WillByDefault(Return(false)); \
    ON_CALL(drivers.refSerial, getRobotData).WillByDefault(ReturnRef(robotData));

#define RUN_EXECUTE_TEST_TURRET_OFFLINE(drivers, chassis, cac, x, y, r)            \
    SET_USER_INPUT(drivers, x, y, r);                                              \
    EXPECT_CALL(                                                                   \
        chassis,                                                                   \
        setDesiredOutput(                                                          \
            FloatNear(x* TEST_WHEEL_SPEED, 1E-3),                                  \
            FloatNear(y* TEST_WHEEL_SPEED, 1E-3),                                  \
            FloatNear(r* TEST_WHEEL_SPEED, 1E-3)));                                \
    ON_CALL(chassis, calculateRotationTranslationalGain).WillByDefault(Return(1)); \
    cac.execute();

TEST(ChassisAutorotateCommand, constructor_only_adds_chassis_sub_req)
{
    DEFAULT_SETUP_TEST(ChassisAutorotateCommand::ChassisSymmetry::SYMMETRICAL_NONE);
    EXPECT_EQ(1U << chassis.getGlobalIdentifier(), cac.getRequirementsBitwise());
}

TEST(ChassisAutorotateCommand, end_sets_chassis_out_0)
{
    DEFAULT_SETUP_TEST(ChassisAutorotateCommand::ChassisSymmetry::SYMMETRICAL_NONE);

    EXPECT_CALL(chassis, setZeroRPM).Times(2);

    cac.end(true);
    cac.end(false);
}

TEST(ChassisAutorotateCommand, isFinished_returns_false)
{
    DEFAULT_SETUP_TEST(ChassisAutorotateCommand::ChassisSymmetry::SYMMETRICAL_NONE);

    EXPECT_FALSE(cac.isFinished());
}

TEST(ChassisAutorotateCommand, execute_turret_offline_chassis_rel_driving_no_autorotate)
{
    DEFAULT_SETUP_TEST(ChassisAutorotateCommand::ChassisSymmetry::SYMMETRICAL_NONE);

    SET_DEFAULT_REF_SERIAL_BEHAVIOR(drivers);

    ON_CALL(turret, isOnline).WillByDefault(Return(false));

    RUN_EXECUTE_TEST_TURRET_OFFLINE(drivers, chassis, cac, 0, 0, 0);
    RUN_EXECUTE_TEST_TURRET_OFFLINE(drivers, chassis, cac, 0, 0, 1);
    RUN_EXECUTE_TEST_TURRET_OFFLINE(drivers, chassis, cac, 1, 1, 0.5);
    RUN_EXECUTE_TEST_TURRET_OFFLINE(drivers, chassis, cac, 0, 0, -0.75);
}

#define SET_TURRET_DEFAULTS(turret, turretAngleActual, turretAngleSetpoint, isYawLimited)      \
    ON_CALL(turret, isOnline).WillByDefault(Return(true));                                     \
    ON_CALL(turret, yawLimited).WillByDefault(Return(isYawLimited));                           \
    ON_CALL(chassis, chassisSpeedRotationPID).WillByDefault([&](float angle, float d) {        \
        return chassis.ChassisSubsystem::chassisSpeedRotationPID(angle, d);                    \
    });                                                                                        \
    ON_CALL(turret, getYawAngleFromCenter).WillByDefault(Return(turretAngleFromCenter));       \
    ContiguousFloat turretAngleActualContiguous(turretAngleActual, 0, 360);                    \
    ON_CALL(turret, getCurrentYawValue).WillByDefault(ReturnRef(turretAngleActualContiguous)); \
    ON_CALL(turret, getYawSetpoint).WillByDefault(Return(turretAngleSetpoint));

static void runExecuteTestSuiteTurretOnlineAtTurretAngle(
    aruwsrc::Drivers& drivers,
    ChassisSubsystemMock& chassis,
    TurretSubsystemMock& turret,
    ChassisAutorotateCommand& cac,
    float turretAngleActual,
    float turretAngleSetpoint,
    bool isYawLimited)
{
    const float turretAngleFromCenter =
        ContiguousFloat(turretAngleActual - TurretSubsystem::YAW_START_ANGLE, -180, 180).getValue();

    SET_TURRET_DEFAULTS(turret, turretAngleActual, turretAngleSetpoint, isYawLimited)

    auto runTest = [&](float x, float y) {
        float rotatedX = x;
        float rotatedY = y;
        rotateVector(&rotatedX, &rotatedY, modm::toRadian(turretAngleFromCenter));
        EXPECT_CALL(
            chassis,
            setDesiredOutput(
                FloatNear(rotatedX * TEST_WHEEL_SPEED, 1E-3),
                FloatNear(rotatedY * TEST_WHEEL_SPEED, 1E-3),
                _));
        SET_USER_INPUT(drivers, x, y, 0);
        ON_CALL(chassis, calculateRotationTranslationalGain).WillByDefault(Return(1));
        cac.execute();
    };

    // Try a bunch of different user inputs
    runTest(0, 0);
    runTest(0.5, 0);
    runTest(0, 0.5);
    runTest(1, 1);
    runTest(-0.5, -0.5);
    runTest(-0.5, 0.25);
}

#define EXECUTE_USER_INPUT_TEST(                                                                                              \
    turretAngleActualStr,                                                                                                     \
    turretAngleActual,                                                                                                        \
    turretAngleSetpointStr,                                                                                                   \
    turretAngleSetpoint,                                                                                                      \
    yawLimited)                                                                                                               \
    TEST(                                                                                                                     \
        ChassisAutorotateCommand,                                                                                             \
        execute_turret_online_turret_rel_driving_actual_##turretAngleActualStr##_deg_setpoint_##turretAngleSetpointStr##_deg) \
    {                                                                                                                         \
        DEFAULT_SETUP_TEST(ChassisAutorotateCommand::ChassisSymmetry::SYMMETRICAL_NONE);                                      \
        SET_DEFAULT_REF_SERIAL_BEHAVIOR(drivers);                                                                             \
        runExecuteTestSuiteTurretOnlineAtTurretAngle(                                                                         \
            drivers,                                                                                                          \
            chassis,                                                                                                          \
            turret,                                                                                                           \
            cac,                                                                                                              \
            turretAngleActual,                                                                                                \
            turretAngleSetpoint,                                                                                              \
            yawLimited);                                                                                                      \
    }

EXECUTE_USER_INPUT_TEST(0, 0, 0, 0, true)
// ensure test passes when setpoint not same as actual
EXECUTE_USER_INPUT_TEST(0, 0, 20, 20, true)
EXECUTE_USER_INPUT_TEST(45, 45, 45, 45, true)
EXECUTE_USER_INPUT_TEST(90, 90, 90, 90, true)
EXECUTE_USER_INPUT_TEST(180, 180, 180, 180, true)
EXECUTE_USER_INPUT_TEST(270, 270, 270, 270, true)
EXECUTE_USER_INPUT_TEST(neg10, -10, neg10, -10, true)
EXECUTE_USER_INPUT_TEST(neg180, -180, neg180, -180, true)
EXECUTE_USER_INPUT_TEST(neg270, -270, neg270, -270, true)
// When angle diff 180 and back/front not identical, test still passes
EXECUTE_USER_INPUT_TEST(0, 0, 180, 180, false)

static void runExecuteAutorotateValidationTest(
    aruwsrc::Drivers& drivers,
    ChassisSubsystemMock& chassis,
    TurretSubsystemMock& turret,
    ChassisAutorotateCommand& cac,
    float turretAngleActual,
    float turretAngleSetpoint,
    bool isYawLimited,
    ChassisAutorotateCommand::ChassisSymmetry chassisSymmetry)
{
    const float turretAngleFromCenter =
        ContiguousFloat(turretAngleActual - TurretSubsystem::YAW_START_ANGLE, -180, 180).getValue();

    SET_TURRET_DEFAULTS(turret, turretAngleActual, turretAngleSetpoint, isYawLimited)
    ON_CALL(chassis, calculateRotationTranslationalGain).WillByDefault(Return(1));
    SET_USER_INPUT(drivers, 0, 0, 0);

    if ((chassisSymmetry != ChassisAutorotateCommand::ChassisSymmetry::SYMMETRICAL_NONE &&
         !isYawLimited &&
         abs(ContiguousFloat(turretAngleActual - turretAngleSetpoint, 0, 360).getValue()) >
             (180 - ChassisAutorotateCommand::SETPOINT_AND_CURRENT_YAW_MATCH_THRESHOLD)) ||
        compareFloatClose(turretAngleFromCenter, 0, 1E-5))
    {
        EXPECT_CALL(chassis, setDesiredOutput(_, _, FloatNear(0.0f, 1E-3)));
        cac.execute();
    }
    else
    {
        EXPECT_CALL(chassis, setDesiredOutput(_, _, testing::Not(0.0f)));
        cac.execute();
    }
}

#define EXECUTE_AUTOROTATE_VALIDATION_TEST(                                                                                                                                           \
    turretAngleActualStr,                                                                                                                                                             \
    turretAngleActual,                                                                                                                                                                \
    turretAngleSetpointStr,                                                                                                                                                           \
    turretAngleSetpoint,                                                                                                                                                              \
    yawLimited,                                                                                                                                                                       \
    chassisSymmetry)                                                                                                                                                                  \
    TEST(                                                                                                                                                                             \
        ChassisAutorotateCommand,                                                                                                                                                     \
        execute_autorotate_validation_turret_actual_##turretAngleActualStr##_deg_setpoint_##turretAngleSetpointStr##_deg_yawLimited_##yawLimited##_chassisSymmetry_##chassisSymmetry) \
    {                                                                                                                                                                                 \
        DEFAULT_SETUP_TEST(chassisSymmetry);                                                                                                                                          \
        SET_DEFAULT_REF_SERIAL_BEHAVIOR(drivers);                                                                                                                                     \
        runExecuteAutorotateValidationTest(                                                                                                                                           \
            drivers,                                                                                                                                                                  \
            chassis,                                                                                                                                                                  \
            turret,                                                                                                                                                                   \
            cac,                                                                                                                                                                      \
            turretAngleActual,                                                                                                                                                        \
            turretAngleSetpoint,                                                                                                                                                      \
            yawLimited,                                                                                                                                                               \
            chassisSymmetry);                                                                                                                                                         \
    }

EXECUTE_AUTOROTATE_VALIDATION_TEST(
    0,
    0,
    0,
    0,
    false,
    ChassisAutorotateCommand::ChassisSymmetry::SYMMETRICAL_NONE)
EXECUTE_AUTOROTATE_VALIDATION_TEST(
    0,
    0,
    0,
    0,
    false,
    ChassisAutorotateCommand::ChassisSymmetry::SYMMETRICAL_180)
EXECUTE_AUTOROTATE_VALIDATION_TEST(
    0,
    0,
    0,
    0,
    true,
    ChassisAutorotateCommand::ChassisSymmetry::SYMMETRICAL_NONE)
EXECUTE_AUTOROTATE_VALIDATION_TEST(
    0,
    0,
    0,
    0,
    true,
    ChassisAutorotateCommand::ChassisSymmetry::SYMMETRICAL_180)

EXECUTE_AUTOROTATE_VALIDATION_TEST(
    0,
    0,
    180,
    180,
    false,
    ChassisAutorotateCommand::ChassisSymmetry::SYMMETRICAL_NONE)
EXECUTE_AUTOROTATE_VALIDATION_TEST(
    0,
    0,
    180,
    180,
    false,
    ChassisAutorotateCommand::ChassisSymmetry::SYMMETRICAL_180)
EXECUTE_AUTOROTATE_VALIDATION_TEST(
    0,
    0,
    180,
    180,
    true,
    ChassisAutorotateCommand::ChassisSymmetry::SYMMETRICAL_NONE)
EXECUTE_AUTOROTATE_VALIDATION_TEST(
    0,
    0,
    180,
    180,
    true,
    ChassisAutorotateCommand::ChassisSymmetry::SYMMETRICAL_180)
