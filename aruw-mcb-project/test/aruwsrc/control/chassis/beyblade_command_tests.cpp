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

#include "aruwsrc/control/chassis/beyblade_command.hpp"
#include "aruwsrc/control/chassis/chassis_subsystem.hpp"
#include "aruwsrc/drivers.hpp"
#include "aruwsrc/mock/chassis_subsystem_mock.hpp"
#include "aruwsrc/mock/turret_subsystem_mock.hpp"

using namespace aruwsrc::chassis;
using namespace aruwsrc::control::turret;
using namespace testing;
using namespace tap::algorithms;
using aruwsrc::mock::ChassisSubsystemMock;
using aruwsrc::mock::TurretSubsystemMock;
using namespace tap::communication::serial;

static constexpr float MAX_R =
    BEYBLADE_ROTATIONAL_SPEED_FRACTION_OF_MAX * CHASSIS_POWER_TO_MAX_SPEED_LUT[0].second;

static constexpr float BASE_DESIRED_OUT =
    CHASSIS_POWER_TO_MAX_SPEED_LUT[0].second * BEYBLADE_TRANSLATIONAL_SPEED_MULTIPLIER;

class BeybladeCommandTest : public Test
{
protected:
    BeybladeCommandTest() : d(), t(&d), cs(&d), bc(&d, &cs, &t) {}

    void SetUp() override
    {
        ON_CALL(cs, getDesiredRotation).WillByDefault(Return(0));
        ON_CALL(t, getYawAngleFromCenter()).WillByDefault(ReturnPointee(&yawAngle));
        ON_CALL(t, isOnline).WillByDefault(Return(true));
        ON_CALL(d.controlOperatorInterface, getChassisXInput()).WillByDefault(ReturnPointee(&x));
        ON_CALL(d.controlOperatorInterface, getChassisYInput()).WillByDefault(ReturnPointee(&y));
        ON_CALL(d.refSerial, getRefSerialReceivingData).WillByDefault(Return(false));
        ON_CALL(cs, calculateRotationTranslationalGain).WillByDefault(Return(1));
        RefSerial::Rx::RobotData rd{};
        ON_CALL(d.refSerial, getRobotData).WillByDefault(ReturnRef(rd));

        bc.initialize();
    }

    void setupDesiredOutputExpectations(float rotation)
    {
        float rotatedX = x;
        float rotatedY = y;
        rotateVector(&rotatedX, &rotatedY, modm::toRadian(yawAngle));
        EXPECT_CALL(
            cs,
            setDesiredOutput(
                FloatNear(BEYBLADE_TRANSLATIONAL_SPEED_MULTIPLIER * rotatedX, 1E-3),
                FloatNear(BEYBLADE_TRANSLATIONAL_SPEED_MULTIPLIER * rotatedY, 1E-3),
                FloatNear(rotation, 1E-3)));
    }

    aruwsrc::Drivers d;
    NiceMock<TurretSubsystemMock> t;
    NiceMock<ChassisSubsystemMock> cs;
    BeybladeCommand bc;
    float yawAngle = 0;
    float x = 0, y = 0;
};

TEST_F(BeybladeCommandTest, execute_all_zeroes_no_ramp)
{
    x = 0, y = 0;

    setupDesiredOutputExpectations(std::min(MAX_R, BEYBLADE_RAMP_UPDATE_RAMP));

    bc.execute();
}

TEST_F(BeybladeCommandTest, execute_positive_xy)
{
    x = BASE_DESIRED_OUT, y = BASE_DESIRED_OUT;

    setupDesiredOutputExpectations(std::min(MAX_R, BEYBLADE_RAMP_UPDATE_RAMP));

    bc.execute();
}

TEST_F(BeybladeCommandTest, execute_negative_xy)
{
    x = -BASE_DESIRED_OUT / 2.0f, y = -BASE_DESIRED_OUT / 4.0f;

    setupDesiredOutputExpectations(std::min(MAX_R, BEYBLADE_RAMP_UPDATE_RAMP));

    bc.execute();
}

TEST_F(BeybladeCommandTest, execute_nonzero_yaw)
{
    x = BASE_DESIRED_OUT, y = -BASE_DESIRED_OUT;
    yawAngle = 90;

    setupDesiredOutputExpectations(std::min(MAX_R, BEYBLADE_RAMP_UPDATE_RAMP));

    bc.execute();
}

TEST_F(BeybladeCommandTest, multiple_executes_beyblade_ramps)
{
    x = BASE_DESIRED_OUT, y = -BASE_DESIRED_OUT;
    yawAngle = 180;

    for (int i = 1; i < 10; i++)
    {
        setupDesiredOutputExpectations(std::min(MAX_R, i * BEYBLADE_RAMP_UPDATE_RAMP));
    }

    for (int i = 1; i < 10; i++)
    {
        bc.execute();
    }
}
