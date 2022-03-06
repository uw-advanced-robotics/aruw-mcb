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

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "tap/algorithms/ramp.hpp"

#include "aruwsrc/control/chassis/beyblade_command.hpp"
#include "aruwsrc/control/chassis/chassis_subsystem.hpp"
#include "aruwsrc/drivers.hpp"
#include "aruwsrc/mock/chassis_subsystem_mock.hpp"
#include "aruwsrc/mock/turret_subsystem_mock.hpp"

#define DEFINE_EXPECTATIONS_FOR_EXECUTE(t, d, cs, baseInput, baseX, baseY, baseR)             \
    ON_CALL(t, getYawAngleFromCenter()).WillByDefault(Return(yawAngle));                      \
    ON_CALL(t, isOnline).WillByDefault(Return(true));                                         \
    ON_CALL(d.controlOperatorInterface, getChassisXInput()).WillByDefault(Return(baseInput)); \
    ON_CALL(d.controlOperatorInterface, getChassisYInput()).WillByDefault(Return(baseInput)); \
    ON_CALL(d.refSerial, getRefSerialReceivingData).WillByDefault(Return(false));             \
    ON_CALL(cs, calculateRotationTranslationalGain).WillByDefault(Return(1));                 \
    RefSerial::Rx::RobotData rd{};                                                            \
    ON_CALL(d.refSerial, getRobotData).WillByDefault(ReturnRef(rd));                          \
    EXPECT_CALL(                                                                              \
        cs,                                                                                   \
        setDesiredOutput(FloatNear(baseX, 1E-3), FloatNear(baseY, 1E-3), FloatNear(baseR, 1E-3)));

using namespace aruwsrc::chassis;
using namespace aruwsrc::control::turret;
using namespace testing;
using aruwsrc::mock::ChassisSubsystemMock;
using aruwsrc::mock::TurretSubsystemMock;
using tap::algorithms::Ramp;
using namespace tap::communication::serial;

static constexpr float BASE_DESIRED_OUT =
    MIN_WHEEL_SPEED_SINGLE_MOTOR * BEYBLADE_TRANSLATIONAL_SPEED_MULTIPLIER;

void basicFrameworkTest(float baseX, float baseY, float maxR, float yawAngle, float baseInput)
{
    aruwsrc::Drivers d;
    NiceMock<TurretSubsystemMock> t(&d);
    NiceMock<ChassisSubsystemMock> cs(&d);
    BeybladeCommand bc(&d, &cs, &t);
    ON_CALL(cs, getDesiredRotation).WillByDefault(Return(0));

    bc.initialize();

    DEFINE_EXPECTATIONS_FOR_EXECUTE(
        t,
        d,
        cs,
        baseInput,
        baseX,
        baseY,
        std::min(maxR, BEYBLADE_RAMP_UPDATE_RMP));

    bc.execute();
}

void basicBigFrameworkTest(float baseX, float baseY, float maxR, float yawAngle, float baseInput)
{
    aruwsrc::Drivers d;
    NiceMock<TurretSubsystemMock> t(&d);
    NiceMock<ChassisSubsystemMock> cs(&d);
    BeybladeCommand bc(&d, &cs, &t);
    ON_CALL(cs, getDesiredRotation).WillByDefault(Return(0));

    bc.initialize();

    for (int i = 1; i <= 10; i++)
    {
        DEFINE_EXPECTATIONS_FOR_EXECUTE(
            t,
            d,
            cs,
            baseInput,
            baseX,
            baseY,
            std::min(maxR, i * BEYBLADE_RAMP_UPDATE_RMP));
        bc.execute();
    }
}

TEST(BeybladeCommand, execute_all_zeroes_no_ramp)
{
    basicFrameworkTest(0, 0, BEYBLADE_POWER_LIMIT_W_TO_ROTATION_TARGET_RPM_LUT[0].second, 0, 0);
}

TEST(BeybladeCommand, execute_fullxy_no_ramp)
{
    basicFrameworkTest(
        BASE_DESIRED_OUT,
        BASE_DESIRED_OUT,
        BEYBLADE_POWER_LIMIT_W_TO_ROTATION_TARGET_RPM_LUT[0].second,
        0,
        1);
}

TEST(BeybladeCommand, execute_fullxy_fullr_180_ramp)
{
    basicFrameworkTest(
        -BASE_DESIRED_OUT,
        -BASE_DESIRED_OUT,
        BEYBLADE_POWER_LIMIT_W_TO_ROTATION_TARGET_RPM_LUT[0].second *
            BEYBLADE_ROTATIONAL_SPEED_CUTOFF_WHEN_TRANSLATING,
        180,
        1);
    basicBigFrameworkTest(
        -BASE_DESIRED_OUT,
        -BASE_DESIRED_OUT,
        BEYBLADE_POWER_LIMIT_W_TO_ROTATION_TARGET_RPM_LUT[0].second *
            BEYBLADE_ROTATIONAL_SPEED_CUTOFF_WHEN_TRANSLATING,
        180,
        1);
}

TEST(BeybladeCommand, execute_halfxy_halfr_270_ramp)
{
    basicFrameworkTest(
        BASE_DESIRED_OUT / 2,
        -BASE_DESIRED_OUT / 2,
        BEYBLADE_POWER_LIMIT_W_TO_ROTATION_TARGET_RPM_LUT[0].second *
            BEYBLADE_ROTATIONAL_SPEED_CUTOFF_WHEN_TRANSLATING,
        270,
        0.5);
    basicBigFrameworkTest(
        -BASE_DESIRED_OUT,
        -BASE_DESIRED_OUT,
        BEYBLADE_POWER_LIMIT_W_TO_ROTATION_TARGET_RPM_LUT[0].second *
            BEYBLADE_ROTATIONAL_SPEED_CUTOFF_WHEN_TRANSLATING,
        180,
        1);
}
