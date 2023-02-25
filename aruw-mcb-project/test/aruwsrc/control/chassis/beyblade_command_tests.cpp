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

#include "tap/drivers.hpp"

#include "aruwsrc/control/chassis/beyblade_command.hpp"
#include "aruwsrc/control/chassis/holonomic_chassis_subsystem.hpp"
#include "aruwsrc/control/chassis/mecanum_chassis_subsystem.hpp"
#include "aruwsrc/mock/chassis_subsystem_mock.hpp"
#include "aruwsrc/mock/control_operator_interface_mock.hpp"
#include "aruwsrc/mock/turret_subsystem_mock.hpp"

using namespace aruwsrc::chassis;
using namespace aruwsrc::control::turret;
using namespace testing;
using namespace tap::algorithms;
using aruwsrc::mock::MecanumChassisSubsystemMock;
using aruwsrc::mock::TurretSubsystemMock;
using namespace tap::communication::serial;

static constexpr float MAX_R =
    BEYBLADE_ROTATIONAL_SPEED_FRACTION_OF_MAX * CHASSIS_POWER_TO_MAX_SPEED_LUT[0].second;

static constexpr float BASE_DESIRED_OUT =
    CHASSIS_POWER_TO_MAX_SPEED_LUT[0].second * BEYBLADE_TRANSLATIONAL_SPEED_MULTIPLIER;

class BeybladeCommandTest : public Test, public WithParamInterface<std::tuple<float, float, float>>
{
protected:
    BeybladeCommandTest()
        : operatorInterface(&d),
          t(&d),
          cs(&d),
          bc(&d, &cs, &t.yawMotor, operatorInterface),
          yawAngle(std::get<2>(GetParam())),
          x(std::get<0>(GetParam())),
          y(std::get<1>(GetParam()))
    {
    }

    void SetUp() override
    {
        ON_CALL(cs, getDesiredRotation).WillByDefault(Return(0));
        ON_CALL(t.yawMotor, getAngleFromCenter).WillByDefault(ReturnPointee(&yawAngle));
        ON_CALL(t.yawMotor, isOnline).WillByDefault(Return(true));
        ON_CALL(operatorInterface, getChassisXInput()).WillByDefault(ReturnPointee(&x));
        ON_CALL(operatorInterface, getChassisYInput()).WillByDefault(ReturnPointee(&y));
        ON_CALL(d.refSerial, getRefSerialReceivingData).WillByDefault(Return(false));
        ON_CALL(cs, calculateRotationTranslationalGain).WillByDefault(Return(1));
        ON_CALL(d.refSerial, getRobotData).WillByDefault(ReturnRef(rd));

        bc.initialize();
    }

    void setupDesiredOutputExpectations(float rotation)
    {
        float rotatedX = x;
        float rotatedY = y;
        rotateVector(&rotatedX, &rotatedY, yawAngle);
        EXPECT_CALL(
            cs,
            setDesiredOutput(
                FloatNear(BEYBLADE_TRANSLATIONAL_SPEED_MULTIPLIER * rotatedX, 1E-3),
                FloatNear(BEYBLADE_TRANSLATIONAL_SPEED_MULTIPLIER * rotatedY, 1E-3),
                FloatNear(rotation, 1E-3)));
    }

    tap::Drivers d;
    NiceMock<aruwsrc::mock::ControlOperatorInterfaceMock> operatorInterface;
    NiceMock<TurretSubsystemMock> t;
    NiceMock<MecanumChassisSubsystemMock> cs;
    BeybladeCommand bc;
    RefSerial::Rx::RobotData rd{};
    float yawAngle = 0;
    float x = 0, y = 0;
};

TEST_P(BeybladeCommandTest, single_execute)
{
    setupDesiredOutputExpectations(std::min(MAX_R, BEYBLADE_RAMP_UPDATE_RAMP));
    bc.execute();
}

TEST_P(BeybladeCommandTest, multiple_execute)
{
    for (int i = 1; i < 10; i++)
    {
        setupDesiredOutputExpectations(std::min(MAX_R, i * BEYBLADE_RAMP_UPDATE_RAMP));
    }

    for (int i = 1; i < 10; i++)
    {
        bc.execute();
    }
}

INSTANTIATE_TEST_SUITE_P(
    BeybladeCommand,
    BeybladeCommandTest,
    Values(
        std::tuple<float, float, float>(0, 0, 0),
        std::tuple<float, float, float>(BASE_DESIRED_OUT, BASE_DESIRED_OUT, 0),
        std::tuple<float, float, float>(BASE_DESIRED_OUT, -BASE_DESIRED_OUT, 0),
        std::tuple<float, float, float>(0, BASE_DESIRED_OUT, 0),
        std::tuple<float, float, float>(-BASE_DESIRED_OUT, BASE_DESIRED_OUT, 0),
        std::tuple<float, float, float>(-BASE_DESIRED_OUT, BASE_DESIRED_OUT, M_PI_2),
        std::tuple<float, float, float>(-BASE_DESIRED_OUT, BASE_DESIRED_OUT, -M_PI_2),
        std::tuple<float, float, float>(-BASE_DESIRED_OUT, BASE_DESIRED_OUT, M_PI)));
