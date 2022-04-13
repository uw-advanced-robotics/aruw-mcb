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
#include "aruwsrc/control/turret/constants/turret_constants.hpp"
#include "aruwsrc/drivers.hpp"
#include "aruwsrc/mock/chassis_subsystem_mock.hpp"
#include "aruwsrc/mock/turret_subsystem_mock.hpp"

using namespace aruwsrc::mock;
using namespace aruwsrc::chassis;
using namespace testing;
using namespace tap::algorithms;
using namespace aruwsrc::control::turret;

class ChassisAutorotateCommandTest : public Test
{
protected:
    ChassisAutorotateCommandTest() : drivers(), chassis(&drivers), turret(&drivers) {}

    void SetUp() override
    {
        ON_CALL(drivers.refSerial, getRefSerialReceivingData).WillByDefault(Return(false));
        ON_CALL(drivers.refSerial, getRobotData).WillByDefault(ReturnRef(robotData));
        ON_CALL(chassis, calculateRotationTranslationalGain).WillByDefault(Return(1));
        ON_CALL(turret.yawMotor, getConfig).WillByDefault(ReturnRef(turretConfig));
    }

    aruwsrc::Drivers drivers;
    NiceMock<ChassisSubsystemMock> chassis;
    NiceMock<TurretSubsystemMock> turret;
    tap::communication::serial::RefSerialData::Rx::RobotData robotData;
    TurretMotorConfig turretConfig;
};

class TurretOfflineTest : public ChassisAutorotateCommandTest,
                          public WithParamInterface<std::tuple<float, float, float>>
{
};

TEST_P(TurretOfflineTest, runExecuteTestTurretOffline)
{
    ChassisAutorotateCommand cac(&drivers, &chassis, &turret.yawMotor);

    ON_CALL(turret.yawMotor, isOnline).WillByDefault(Return(false));

    ON_CALL(drivers.controlOperatorInterface, getChassisXInput)
        .WillByDefault(Return(std::get<0>(GetParam())));
    ON_CALL(drivers.controlOperatorInterface, getChassisYInput)
        .WillByDefault(Return(std::get<1>(GetParam())));
    ON_CALL(drivers.controlOperatorInterface, getChassisRInput)
        .WillByDefault(Return(std::get<2>(GetParam())));

    EXPECT_CALL(
        chassis,
        setDesiredOutput(
            FloatNear(std::get<0>(GetParam()), 1E-3),
            FloatNear(std::get<1>(GetParam()), 1E-3),
            FloatNear(std::get<2>(GetParam()), 1E-3)));

    cac.execute();
}

TEST_F(ChassisAutorotateCommandTest, constructor_only_adds_chassis_sub_req)
{
    ChassisAutorotateCommand cac(&drivers, &chassis, &turret.yawMotor);
    EXPECT_EQ(1U << chassis.getGlobalIdentifier(), cac.getRequirementsBitwise());
}

TEST_F(ChassisAutorotateCommandTest, end_sets_chassis_out_0)
{
    ChassisAutorotateCommand cac(&drivers, &chassis, &turret.yawMotor);

    EXPECT_CALL(chassis, setZeroRPM).Times(2);

    cac.end(true);
    cac.end(false);
}

TEST_F(ChassisAutorotateCommandTest, isFinished_returns_false)
{
    ChassisAutorotateCommand cac(&drivers, &chassis, &turret.yawMotor);

    EXPECT_FALSE(cac.isFinished());
}

INSTANTIATE_TEST_SUITE_P(
    ChassisAutorotateCommand,
    TurretOfflineTest,
    Values(
        std::tuple<float, float, float>(0, 0, 0),
        std::tuple<float, float, float>(1000, 0, 1000),
        std::tuple<float, float, float>(-1000, -1000, 1000),
        std::tuple<float, float, float>(0, 1000, -1000)));

struct TurretOnlineTestStruct
{
    float x = 0;
    float y = 0;
    float r = 0;
    float yawAngle = 0;
    ContiguousFloat yawSetpoint = ContiguousFloat(0, 0, M_TWOPI);
    bool yawLimited = false;
    ChassisAutorotateCommand::ChassisSymmetry chassisSymmetry =
        ChassisAutorotateCommand::ChassisSymmetry::SYMMETRICAL_NONE;
    uint8_t padding[2] = {};  // padding to avoid valgrind errors
};

class TurretOnlineTest : public ChassisAutorotateCommandTest,
                         public WithParamInterface<TurretOnlineTestStruct>
{
public:
    TurretOnlineTest()
        : yawAngleFromCenter(ContiguousFloat(
                                 GetParam().yawAngle - turret.yawMotor.getConfig().startAngle,
                                 -M_PI,
                                 M_PI)
                                 .getValue()),
          cac(&drivers, &chassis, &turret.yawMotor, GetParam().chassisSymmetry),
          turretAngleActualContiguous(GetParam().yawAngle, 0, 360)
    {
    }

    void SetUp() override
    {
        ChassisAutorotateCommandTest::SetUp();

        ON_CALL(drivers.mpu6500, getGz).WillByDefault(Return(0));

        ON_CALL(drivers.controlOperatorInterface, getChassisXInput)
            .WillByDefault(Return(GetParam().x));
        ON_CALL(drivers.controlOperatorInterface, getChassisYInput)
            .WillByDefault(Return(GetParam().y));
        ON_CALL(drivers.controlOperatorInterface, getChassisRInput)
            .WillByDefault(Return(GetParam().r));

        turretConfig.limitMotorAngles = GetParam().yawLimited;

        ON_CALL(turret.yawMotor, isOnline).WillByDefault(Return(true));
        ON_CALL(turret.yawMotor, getAngleFromCenter).WillByDefault(Return(yawAngleFromCenter));
        ON_CALL(turret.yawMotor, getChassisFrameVelocity).WillByDefault(Return(0));
        ON_CALL(turret.yawMotor, getChassisFrameMeasuredAngle)
            .WillByDefault(ReturnRef(turretAngleActualContiguous));
        ON_CALL(turret.yawMotor, getChassisFrameSetpoint)
            .WillByDefault(ReturnRef(GetParam().yawSetpoint));

        ON_CALL(chassis, chassisSpeedRotationPID).WillByDefault([&](float angle, float d) {
            return chassis.ChassisSubsystem::chassisSpeedRotationPID(angle, d);
        });
    }

    float yawAngleFromCenter;

    ChassisAutorotateCommand cac;
    ContiguousFloat turretAngleActualContiguous;
};

TEST_P(TurretOnlineTest, execute_rotated_xy)
{
    float rotatedX = GetParam().x;
    float rotatedY = GetParam().y;
    rotateVector(&rotatedX, &rotatedY, modm::toRadian(yawAngleFromCenter));

    EXPECT_CALL(chassis, setDesiredOutput(FloatNear(rotatedX, 1E-3), FloatNear(rotatedY, 1E-3), _));

    cac.execute();
}

TEST_P(TurretOnlineTest, execute_autorotation_works)
{
    bool shouldAutorotate = true;

    std::vector<float> yawAnglesWhereNoAutorotation{0};

    if (!GetParam().yawLimited &&
        GetParam().chassisSymmetry == ChassisAutorotateCommand::ChassisSymmetry::SYMMETRICAL_90)
    {
        yawAnglesWhereNoAutorotation.push_back(-90);
        yawAnglesWhereNoAutorotation.push_back(90);
        yawAnglesWhereNoAutorotation.push_back(-180);
        yawAnglesWhereNoAutorotation.push_back(180);
    }
    else if (
        !GetParam().yawLimited &&
        GetParam().chassisSymmetry == ChassisAutorotateCommand::ChassisSymmetry::SYMMETRICAL_180)
    {
        yawAnglesWhereNoAutorotation.push_back(-180);
        yawAnglesWhereNoAutorotation.push_back(180);
    }

    for (float val : yawAnglesWhereNoAutorotation)
    {
        if (compareFloatClose(val, yawAngleFromCenter, 1E-5))
        {
            shouldAutorotate = false;
        }
    }

    if (GetParam().chassisSymmetry != ChassisAutorotateCommand::ChassisSymmetry::SYMMETRICAL_NONE &&
        !GetParam().yawLimited &&
        turretAngleActualContiguous.difference(GetParam().yawSetpoint) >
            (180 - ChassisAutorotateCommand::TURRET_YAW_SETPOINT_MEAS_DIFF_TO_APPLY_AUTOROTATION))
    {
        shouldAutorotate = false;
    }

    if (shouldAutorotate)
    {
        EXPECT_CALL(chassis, setDesiredOutput(_, _, Not(0.0f)));
    }
    else
    {
        EXPECT_CALL(chassis, setDesiredOutput(_, _, FloatNear(0.0f, 1E-3)));
    }

    cac.execute();
}

// INSTANTIATE_TEST_SUITE_P(
//     ChassisAutorotateCommand,
//     TurretOnlineTest,
//     Values(
//         TurretOnlineTestStruct{
//             .x = 0,
//             .y = 0,
//             .r = 0,
//             .yawAngle = M_PI_4,
//             .yawSetpoint = ContiguousFloat(0, 0, M_TWOPI),
//             .yawLimited = true,
//             .chassisSymmetry = ChassisAutorotateCommand::ChassisSymmetry::SYMMETRICAL_NONE,
//         },
//         TurretOnlineTestStruct{
//             .x = 10,
//             .y = 10,
//             .r = 10,
//             .yawAngle = M_PI_2,
//             .yawSetpoint = ContiguousFloat(0, 0, M_TWOPI),
//             .yawLimited = true,
//             .chassisSymmetry = ChassisAutorotateCommand::ChassisSymmetry::SYMMETRICAL_NONE,
//         },
//         TurretOnlineTestStruct{
//             .x = -10,
//             .y = -10,
//             .r = -10,
//             .yawAngle = -M_PI_4,
//             .yawSetpoint = ContiguousFloat(0, 0, M_TWOPI),
//             .yawLimited = true,
//             .chassisSymmetry = ChassisAutorotateCommand::ChassisSymmetry::SYMMETRICAL_NONE,
//         },
//         TurretOnlineTestStruct{
//             .x = -10,
//             .y = 0,
//             .r = 10,
//             .yawAngle = modm::toRadian(-135),
//             .yawSetpoint = ContiguousFloat(0, 0, M_TWOPI),
//             .yawLimited = true,
//             .chassisSymmetry = ChassisAutorotateCommand::ChassisSymmetry::SYMMETRICAL_NONE,
//         },
//         TurretOnlineTestStruct{
//             .x = 0,
//             .y = 0,
//             .r = 0,
//             .yawAngle = -M_PI,
//             .yawSetpoint = ContiguousFloat(0, 0, M_TWOPI),
//             .yawLimited = true,
//             .chassisSymmetry = ChassisAutorotateCommand::ChassisSymmetry::SYMMETRICAL_NONE,
//         },
//         TurretOnlineTestStruct{
//             .x = 0,
//             .y = 0,
//             .r = 0,
//             .yawAngle = 0,
//             .yawSetpoint = ContiguousFloat(M_PI, 0, M_TWOPI),
//             .yawLimited = true,
//             .chassisSymmetry = ChassisAutorotateCommand::ChassisSymmetry::SYMMETRICAL_180,
//         },
//         TurretOnlineTestStruct{
//             .x = 0,
//             .y = 0,
//             .r = 0,
//             .yawAngle = M_PI
//             .yawSetpoint = ContiguousFloat(M_PI, 0, M_TWOPI),
//             .yawLimited = false,
//             .chassisSymmetry = ChassisAutorotateCommand::ChassisSymmetry::SYMMETRICAL_180,
//         },
//         TurretOnlineTestStruct{
//             .x = 0,
//             .y = 0,
//             .r = 0,
//             .yawAngle = -180,
//             .yawSetpoint = ContiguousFloat(0, 0, M_TWOPI),
//             .yawLimited = false,
//             .chassisSymmetry = ChassisAutorotateCommand::ChassisSymmetry::SYMMETRICAL_90,
//         },
//         TurretOnlineTestStruct{
//             .x = 0,
//             .y = 0,
//             .r = 0,
//             .yawAngle = 45,
//             .yawSetpoint = ContiguousFloat(-M_PI_4, 0, M_TWOPI),
//             .yawLimited = true,
//             .chassisSymmetry = ChassisAutorotateCommand::ChassisSymmetry::SYMMETRICAL_90,
//         },
//         TurretOnlineTestStruct{
//             .x = 0,
//             .y = 0,
//             .r = 0,
//             .yawAngle = 0,
//             .yawSetpoint = ContiguousFloat(M_PI_2, 0, M_TWOPI),
//             .yawLimited = false,
//             .chassisSymmetry = ChassisAutorotateCommand::ChassisSymmetry::SYMMETRICAL_90,
//         }),
//     [](const ::testing::TestParamInfo<TurretOnlineTest::ParamType>& info) {
//         std::stringstream ss;
//         ss << "x_" << PrintToString(info.param.x) << "_y_" << PrintToString(info.param.y) <<
//         "_r_"
//            << PrintToString(info.param.r) << "_yawAngle_" << PrintToString(info.param.yawAngle)
//            << "_yawSetpoint_" << PrintToString(info.param.yawSetpoint.getValue()) <<
//            "_yawLimited_"
//            << PrintToString(info.param.yawLimited) << "_chassisSymmetry_"
//            << PrintToString(static_cast<int>(info.param.chassisSymmetry));
//         std::string s = ss.str();
//         std::replace(s.begin(), s.end(), '-', '_');
//         return s;
//     });
