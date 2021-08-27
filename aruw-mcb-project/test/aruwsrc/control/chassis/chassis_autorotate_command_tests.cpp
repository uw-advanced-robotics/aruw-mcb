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

#include "aruwsrc/control/chassis/chassis_autorotate_command.hpp"
#include "aruwsrc/mock/chassis_subsystem_mock.hpp"
#include "aruwsrc/mock/turret_subsystem_mock.hpp"

using namespace tap;
using namespace aruwsrc::mock;
using namespace aruwsrc::chassis;
using namespace testing;
using namespace tap::algorithms;

static constexpr float MAX_WHEEL_SPEED = ChassisSubsystem::MAX_WHEEL_SPEED_SINGLE_MOTOR;
static constexpr float AUTOROTATE_KP = ChassisAutorotateCommand::CHASSIS_AUTOROTATE_PID_KP;

#define DEFAULT_SETUP_TEST()                          \
    Drivers drivers;                                  \
    NiceMock<ChassisSubsystemMock> chassis(&drivers); \
    NiceMock<TurretSubsystemMock> turret(&drivers);   \
    ChassisAutorotateCommand cac(&drivers, &chassis, &turret);

#define SET_USER_INPUT(drivers, x, y, r)                                                  \
    ON_CALL(drivers.controlOperatorInterface, getChassisXInput).WillByDefault(Return(x)); \
    ON_CALL(drivers.controlOperatorInterface, getChassisYInput).WillByDefault(Return(y)); \
    ON_CALL(drivers.controlOperatorInterface, getChassisRInput).WillByDefault(Return(r));

#define RUN_EXECUTE_TEST_TURRET_OFFLINE(drivers, chassis, cac, x, y, r)            \
    SET_USER_INPUT(drivers, x, y, r);                                              \
    EXPECT_CALL(                                                                   \
        chassis,                                                                   \
        setDesiredOutput(                                                          \
            FloatEq(x* MAX_WHEEL_SPEED),                                           \
            FloatEq(y* MAX_WHEEL_SPEED),                                           \
            FloatEq(r* MAX_WHEEL_SPEED)));                                         \
    ON_CALL(chassis, calculateRotationTranslationalGain).WillByDefault(Return(1)); \
    cac.execute();

TEST(ChassisAutorotateCommand, constructor_only_adds_chassis_sub_req)
{
    DEFAULT_SETUP_TEST();
    EXPECT_EQ(1U << chassis.getGlobalIdentifier(), cac.getRequirementsBitwise());
}

TEST(ChassisAutorotateCommand, end_sets_chassis_out_0)
{
    DEFAULT_SETUP_TEST();

    EXPECT_CALL(chassis, setDesiredOutput(0, 0, 0)).Times(2);

    cac.end(true);
    cac.end(false);
}

TEST(ChassisAutorotateCommand, isFinished_returns_false)
{
    DEFAULT_SETUP_TEST();

    EXPECT_FALSE(cac.isFinished());
}

TEST(ChassisAutorotateCommand, execute_turret_offline_chassis_rel_driving_no_autorotate)
{
    DEFAULT_SETUP_TEST();

    ON_CALL(turret, isOnline).WillByDefault(Return(false));

    RUN_EXECUTE_TEST_TURRET_OFFLINE(drivers, chassis, cac, 0, 0, 0);
    RUN_EXECUTE_TEST_TURRET_OFFLINE(drivers, chassis, cac, 0, 0, 1);
    RUN_EXECUTE_TEST_TURRET_OFFLINE(drivers, chassis, cac, 1, 1, 0.5);
    RUN_EXECUTE_TEST_TURRET_OFFLINE(drivers, chassis, cac, 0, 0, -0.75);
}

static void runExecuteTestSuiteTurretOnlineAtTurretAngle(
    Drivers& drivers,
    ChassisSubsystemMock& chassis,
    TurretSubsystemMock& turret,
    ChassisAutorotateCommand& cac,
    float turretAngleFromCenter)
{
    ON_CALL(chassis, chassisSpeedRotationPID).WillByDefault([&](float angle, float kp) {
        return chassis.ChassisSubsystem::chassisSpeedRotationPID(angle, kp);
    });

    ON_CALL(turret, getYawAngleFromCenter).WillByDefault(Return(turretAngleFromCenter));

    auto runTest = [&](float x, float y) {
        float rotatedX = x;
        float rotatedY = y;
        rotateVector(&rotatedX, &rotatedY, -degreesToRadians(turretAngleFromCenter));
        EXPECT_CALL(
            chassis,
            setDesiredOutput(
                FloatEq(rotatedX * MAX_WHEEL_SPEED),
                FloatEq(rotatedY * MAX_WHEEL_SPEED),
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

#define EXECUTE_ANGLE_TEST(angleStr, angle)                                                   \
    TEST(ChassisAutorotateCommand, execute_turret_online_turret_rel_driving_##angleStr##_deg) \
    {                                                                                         \
        DEFAULT_SETUP_TEST();                                                                 \
        ON_CALL(turret, isOnline).WillByDefault(Return(true));                                \
        runExecuteTestSuiteTurretOnlineAtTurretAngle(drivers, chassis, turret, cac, angle);   \
    }

EXECUTE_ANGLE_TEST(0, 0)
EXECUTE_ANGLE_TEST(45, 45)
EXECUTE_ANGLE_TEST(90, 90)
EXECUTE_ANGLE_TEST(180, 180)
EXECUTE_ANGLE_TEST(270, 270)
EXECUTE_ANGLE_TEST(neg10, -10)
EXECUTE_ANGLE_TEST(neg180, -180)
EXECUTE_ANGLE_TEST(neg270, -270)
