/*
 * Copyright (c) 2020-2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "tap/communication/sensors/current/analog_current_sensor.hpp"
#include "tap/drivers.hpp"

#include "aruwsrc/communication/sensors/current/acs712_current_sensor_config.hpp"
#include "aruwsrc/control/chassis/chassis_autorotate_command.hpp"
#include "aruwsrc/control/chassis/mecanum_chassis_subsystem.hpp"
#include "aruwsrc/control/turret/constants/turret_constants.hpp"
#include "aruwsrc/mock/control_operator_interface_mock.hpp"
#include "aruwsrc/mock/mecanum_chassis_subsystem_mock.hpp"
#include "aruwsrc/mock/turret_subsystem_mock.hpp"

using namespace aruwsrc::mock;
using namespace aruwsrc::chassis;
using namespace testing;
using namespace tap::algorithms;
using namespace aruwsrc::control::turret;

class ChassisAutorotateCommandTest : public Test
{
protected:
    ChassisAutorotateCommandTest()
        : drivers(),
          currentSensor(
              {&drivers.analog,
               aruwsrc::chassis::CURRENT_SENSOR_PIN,
               aruwsrc::communication::sensors::current::ACS712_CURRENT_SENSOR_MV_PER_MA,
               aruwsrc::communication::sensors::current::ACS712_CURRENT_SENSOR_ZERO_MA,
               aruwsrc::communication::sensors::current::ACS712_CURRENT_SENSOR_LOW_PASS_ALPHA}),
          chassis(&drivers, &currentSensor),
          turret(&drivers),
          controlOperatorInterface(&drivers),
          turretConfig{0, 0, 0, M_PI, false}
    {
    }

    void SetUp() override
    {
        ON_CALL(drivers.refSerial, getRefSerialReceivingData).WillByDefault(Return(false));
        ON_CALL(drivers.refSerial, getRobotData).WillByDefault(ReturnRef(robotData));
        ON_CALL(chassis, calculateRotationTranslationalGain).WillByDefault(Return(1));
        ON_CALL(turret.yawMotor, getConfig).WillByDefault(ReturnRef(turretConfig));
    }

    tap::Drivers drivers;
    tap::communication::sensors::current::AnalogCurrentSensor currentSensor;
    NiceMock<MecanumChassisSubsystemMock> chassis;
    NiceMock<TurretSubsystemMock> turret;
    NiceMock<ControlOperatorInterfaceMock> controlOperatorInterface;
    tap::communication::serial::RefSerialData::Rx::RobotData robotData;
    TurretMotorConfig turretConfig;
};

class TurretOfflineTest : public ChassisAutorotateCommandTest,
                          public WithParamInterface<std::tuple<float, float, float>>
{
};

TEST_P(TurretOfflineTest, runExecuteTestTurretOffline)
{
    ChassisAutorotateCommand cac(&drivers, &(controlOperatorInterface), &chassis, &turret.yawMotor);

    ON_CALL(turret.yawMotor, isOnline).WillByDefault(Return(false));

    ON_CALL(controlOperatorInterface, getChassisXInput)
        .WillByDefault(Return(std::get<0>(GetParam())));
    ON_CALL(controlOperatorInterface, getChassisYInput)
        .WillByDefault(Return(std::get<1>(GetParam())));
    ON_CALL(controlOperatorInterface, getChassisRInput)
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
    ChassisAutorotateCommand cac(&drivers, &(controlOperatorInterface), &chassis, &turret.yawMotor);
    EXPECT_EQ(1U << chassis.getGlobalIdentifier(), cac.getRequirementsBitwise());
}

TEST_F(ChassisAutorotateCommandTest, end_sets_chassis_out_0)
{
    ChassisAutorotateCommand cac(&drivers, &(controlOperatorInterface), &chassis, &turret.yawMotor);

    EXPECT_CALL(chassis, setZeroRPM).Times(2);

    cac.end(true);
    cac.end(false);
}

TEST_F(ChassisAutorotateCommandTest, isFinished_returns_false)
{
    ChassisAutorotateCommand cac(&drivers, &(controlOperatorInterface), &chassis, &turret.yawMotor);

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
    WrappedFloat yawSetpoint = Angle(0);
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
        : yawAngleFromCenter(WrappedFloat(
                                 GetParam().yawAngle - turret.yawMotor.getConfig().startAngle,
                                 -M_PI,
                                 M_PI)
                                 .getWrappedValue()),
          cac(&drivers,
              &(controlOperatorInterface),
              &chassis,
              &turret.yawMotor,
              GetParam().chassisSymmetry),
          turretAngleActual(GetParam().yawAngle, 0, M_TWOPI)
    {
    }

    void SetUp() override
    {
        ChassisAutorotateCommandTest::SetUp();

        ON_CALL(drivers.mpu6500, getGz).WillByDefault(Return(0));

        ON_CALL(controlOperatorInterface, getChassisXInput).WillByDefault(Return(GetParam().x));
        ON_CALL(controlOperatorInterface, getChassisYInput).WillByDefault(Return(GetParam().y));
        ON_CALL(controlOperatorInterface, getChassisRInput).WillByDefault(Return(GetParam().r));

        turretConfig.limitMotorAngles = GetParam().yawLimited;

        ON_CALL(turret.yawMotor, isOnline).WillByDefault(Return(true));
        ON_CALL(turret.yawMotor, getAngleFromCenter).WillByDefault(Return(yawAngleFromCenter));
        ON_CALL(turret.yawMotor, getChassisFrameVelocity).WillByDefault(Return(0));
        ON_CALL(turret.yawMotor, getChassisFrameMeasuredAngle)
            .WillByDefault(ReturnRef(turretAngleActual));
        ON_CALL(turret.yawMotor, getChassisFrameSetpoint)
            .WillByDefault(ReturnPointee(&GetParam().yawSetpoint));

        ON_CALL(chassis, chassisSpeedRotationPID)
            .WillByDefault(
                [&](float angle, float d)
                { return chassis.HolonomicChassisSubsystem::chassisSpeedRotationPID(angle, d); });
    }

    float yawAngleFromCenter;

    ChassisAutorotateCommand cac;
    WrappedFloat turretAngleActual;
};

TEST_P(TurretOnlineTest, execute_rotated_xy)
{
    float rotatedX = GetParam().x;
    float rotatedY = GetParam().y;
    rotateVector(&rotatedX, &rotatedY, yawAngleFromCenter);

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
        yawAnglesWhereNoAutorotation.push_back(-M_PI_2);
        yawAnglesWhereNoAutorotation.push_back(M_PI_2);
        yawAnglesWhereNoAutorotation.push_back(-M_PI);
        yawAnglesWhereNoAutorotation.push_back(M_PI);
    }
    else if (
        !GetParam().yawLimited &&
        GetParam().chassisSymmetry == ChassisAutorotateCommand::ChassisSymmetry::SYMMETRICAL_180)
    {
        yawAnglesWhereNoAutorotation.push_back(-M_PI);
        yawAnglesWhereNoAutorotation.push_back(M_PI);
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
        turretAngleActual.minDifference(GetParam().yawSetpoint) >
            (M_PI - ChassisAutorotateCommand::TURRET_YAW_SETPOINT_MEAS_DIFF_TO_APPLY_AUTOROTATION))
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

INSTANTIATE_TEST_SUITE_P(
    ChassisAutorotateCommand,
    TurretOnlineTest,
    Values(
        TurretOnlineTestStruct{
            .x = 0,
            .y = 0,
            .r = 0,
            .yawAngle = M_PI_4,
            .yawSetpoint = Angle(0),
            .yawLimited = true,
            .chassisSymmetry = ChassisAutorotateCommand::ChassisSymmetry::SYMMETRICAL_NONE,
        },
        TurretOnlineTestStruct{
            .x = 10,
            .y = 10,
            .r = 10,
            .yawAngle = M_PI_2,
            .yawSetpoint = Angle(0),
            .yawLimited = true,
            .chassisSymmetry = ChassisAutorotateCommand::ChassisSymmetry::SYMMETRICAL_NONE,
        },
        TurretOnlineTestStruct{
            .x = -10,
            .y = -10,
            .r = -10,
            .yawAngle = -M_PI_4,
            .yawSetpoint = Angle(0),
            .yawLimited = true,
            .chassisSymmetry = ChassisAutorotateCommand::ChassisSymmetry::SYMMETRICAL_NONE,
        },
        TurretOnlineTestStruct{
            .x = -10,
            .y = 0,
            .r = 10,
            .yawAngle = modm::toRadian(-135),
            .yawSetpoint = Angle(0),
            .yawLimited = true,
            .chassisSymmetry = ChassisAutorotateCommand::ChassisSymmetry::SYMMETRICAL_NONE,
        },
        TurretOnlineTestStruct{
            .x = 0,
            .y = 0,
            .r = 0,
            .yawAngle = -M_PI,
            .yawSetpoint = Angle(0),
            .yawLimited = true,
            .chassisSymmetry = ChassisAutorotateCommand::ChassisSymmetry::SYMMETRICAL_NONE,
        },
        TurretOnlineTestStruct{
            .x = 0,
            .y = 0,
            .r = 0,
            .yawAngle = 0,
            .yawSetpoint = Angle(M_PI),
            .yawLimited = true,
            .chassisSymmetry = ChassisAutorotateCommand::ChassisSymmetry::SYMMETRICAL_180,
        },
        TurretOnlineTestStruct{
            .x = 0,
            .y = 0,
            .r = 0,
            .yawAngle = M_PI,
            .yawSetpoint = Angle(M_PI),
            .yawLimited = false,
            .chassisSymmetry = ChassisAutorotateCommand::ChassisSymmetry::SYMMETRICAL_180,
        },
        TurretOnlineTestStruct{
            .x = 0,
            .y = 0,
            .r = 0,
            .yawAngle = -M_PI,
            .yawSetpoint = Angle(0),
            .yawLimited = false,
            .chassisSymmetry = ChassisAutorotateCommand::ChassisSymmetry::SYMMETRICAL_90,
        },
        TurretOnlineTestStruct{
            .x = 0,
            .y = 0,
            .r = 0,
            .yawAngle = M_PI_4,
            .yawSetpoint = Angle(-M_PI_4),
            .yawLimited = true,
            .chassisSymmetry = ChassisAutorotateCommand::ChassisSymmetry::SYMMETRICAL_90,
        },
        TurretOnlineTestStruct{
            .x = 0,
            .y = 0,
            .r = 0,
            .yawAngle = 0,
            .yawSetpoint = Angle(M_PI_2),
            .yawLimited = false,
            .chassisSymmetry = ChassisAutorotateCommand::ChassisSymmetry::SYMMETRICAL_90,
        }),
    [](const ::testing::TestParamInfo<TurretOnlineTest::ParamType>& info)
    {
        std::stringstream ss;
        ss << "x_" << PrintToString(info.param.x) << "_y_" << PrintToString(info.param.y) << "_r_"
           << PrintToString(info.param.r) << "_yawAngle_"
           << PrintToString(static_cast<int>(modm::toDegree(info.param.yawAngle)))
           << "_yawSetpoint_"
           << PrintToString(
                  static_cast<int>(modm::toDegree(info.param.yawSetpoint.getUnwrappedValue())))
           << "_yawLimited_" << PrintToString(info.param.yawLimited) << "_chassisSymmetry_"
           << PrintToString(static_cast<int>(info.param.chassisSymmetry));
        std::string s = ss.str();
        std::replace(s.begin(), s.end(), '-', '_');
        return s;
    });
