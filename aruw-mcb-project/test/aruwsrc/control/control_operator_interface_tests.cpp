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

/*
 * Note: These tests do not fully test any underlying filter structures
 * For example, there are no long term tests to ensure linear interpolation
 * is done properly for the chassis remote input. Instead, it is expected
 * that additional tests should be done to ensure linear interpolation is
 * done properly.
 */

#include <gtest/gtest.h>

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/architecture/clock.hpp"
#include "tap/drivers.hpp"

#include "aruwsrc/control/chassis/constants/chassis_constants.hpp"
#include "aruwsrc/robot/control_operator_interface.hpp"

using aruwsrc::control::ControlOperatorInterface;
using namespace tap::communication::serial;
using namespace testing;
using namespace tap::arch::clock;
using namespace tap::algorithms;

class ControlOperatorInterfaceTest : public Test
{
protected:
    ControlOperatorInterfaceTest() : operatorInterface(&drivers) {}

    void SetUp() override
    {
        ON_CALL(drivers.refSerial, getRobotData).WillByDefault(ReturnRef(robotData));
        ON_CALL(drivers.refSerial, getRefSerialReceivingData).WillByDefault(Return(false));
        ON_CALL(drivers.remote, getUpdateCounter).WillByDefault(ReturnPointee(&updateCounter));
    }

    tap::Drivers drivers;
    ClockStub clock;
    ControlOperatorInterface operatorInterface;
    tap::communication::serial::RefSerialData::Rx::RobotData robotData;
    uint32_t updateCounter = 0;
};

static constexpr float MAX_CHASSIS_SPEED =
    aruwsrc::chassis::CHASSIS_POWER_TO_MAX_SPEED_LUT[0].second;

using COIChassisTuple = std::tuple<float, bool, bool, float>;

class ChassisTest : public ControlOperatorInterfaceTest, public WithParamInterface<COIChassisTuple>
{
};

TEST_P(ChassisTest, getChassisXInput_settles_to_des_rpm)
{
    auto params = GetParam();

    float remoteVal = std::get<0>(params);
    bool wPressed = std::get<1>(params);
    bool sPressed = std::get<2>(params);
    float expectedValue = std::get<3>(params);

    ON_CALL(drivers.remote, keyPressed(Remote::Key::W)).WillByDefault(Return(wPressed));
    ON_CALL(drivers.remote, keyPressed(Remote::Key::S)).WillByDefault(Return(sPressed));
    ON_CALL(drivers.remote, getChannel(Remote::Channel::LEFT_VERTICAL))
        .WillByDefault(Return(remoteVal));

    // Since acceleration schemes are volatile, we only care about steady state behavior of the
    // function, so call it a bunch to allow it to settle to the expected value
    for (int i = 0; i < 10; i++)
    {
        clock.time += 1'000;
        updateCounter++;
        operatorInterface.getChassisXInput();
    }

    EXPECT_NEAR(expectedValue, operatorInterface.getChassisXInput(), 1E-3);
}

TEST_P(ChassisTest, getChassisYInput_settles_to_des_rpm)
{
    auto params = GetParam();

    float remoteVal = std::get<0>(params);
    bool aPressed = std::get<1>(params);
    bool dPressed = std::get<2>(params);
    float expectedValue = std::get<3>(params);

    ON_CALL(drivers.remote, keyPressed(Remote::Key::A)).WillByDefault(Return(aPressed));
    ON_CALL(drivers.remote, keyPressed(Remote::Key::D)).WillByDefault(Return(dPressed));
    ON_CALL(drivers.remote, getChannel(Remote::Channel::LEFT_HORIZONTAL))
        .WillByDefault(Return(-remoteVal));

    for (int i = 0; i < 10; i++)
    {
        clock.time += 1'000;
        updateCounter++;
        operatorInterface.getChassisYInput();
    }

    EXPECT_NEAR(expectedValue, operatorInterface.getChassisYInput(), 1E-3);
}

TEST_P(ChassisTest, getChassisRInput_settles_to_des_rpm)
{
    auto params = GetParam();

    float remoteVal = std::get<0>(params);
    bool qPressed = std::get<1>(params);
    bool ePressed = std::get<2>(params);
    float expectedValue = std::get<3>(params);

    ON_CALL(drivers.remote, keyPressed(Remote::Key::Q)).WillByDefault(Return(qPressed));
    ON_CALL(drivers.remote, keyPressed(Remote::Key::E)).WillByDefault(Return(ePressed));
    ON_CALL(drivers.remote, getChannel(Remote::Channel::RIGHT_HORIZONTAL))
        .WillByDefault(Return(-remoteVal));

    for (int i = 0; i < 10; i++)
    {
        clock.time += 1'000;
        updateCounter++;
        operatorInterface.getChassisRInput();
    }

    EXPECT_NEAR(expectedValue, operatorInterface.getChassisRInput(), 1E-3);
}

INSTANTIATE_TEST_SUITE_P(
    ControlOperatorInterface,
    ChassisTest,
    Values(
        COIChassisTuple(0, false, false, 0),
        COIChassisTuple(0, true, false, MAX_CHASSIS_SPEED),
        COIChassisTuple(0, true, true, 0),
        COIChassisTuple(0, false, true, -MAX_CHASSIS_SPEED),
        COIChassisTuple(1, false, false, MAX_CHASSIS_SPEED),
        COIChassisTuple(1, false, true, 0),
        COIChassisTuple(0.5, false, false, 0.5f * MAX_CHASSIS_SPEED)));

class TurretTest : public ControlOperatorInterfaceTest,
                   public WithParamInterface<std::tuple<float, int16_t, float>>
{
};

TEST_P(TurretTest, getTurretYawInput_returns_user_input)
{
    auto params = GetParam();

    float remoteInput = std::get<0>(params);
    int16_t mouseInput = std::get<1>(params);
    float expectedValue = std::get<2>(params);

    EXPECT_CALL(drivers.remote, getMouseX).WillOnce(Return(mouseInput));
    EXPECT_CALL(drivers.remote, getChannel(Remote::Channel::RIGHT_HORIZONTAL))
        .WillOnce(Return(remoteInput));

    EXPECT_NEAR(expectedValue, operatorInterface.getTurretYawInput(0), 1E-3);
}

TEST_P(TurretTest, getTurretYawInput_turret2_returns_user_input)
{
    auto params = GetParam();

    float remoteInput = std::get<0>(params);
    int16_t mouseInput = std::get<1>(params);
    float expectedValue = std::get<2>(params);

    EXPECT_CALL(drivers.remote, getMouseX).WillOnce(Return(mouseInput));
    EXPECT_CALL(drivers.remote, getChannel(Remote::Channel::LEFT_HORIZONTAL))
        .WillOnce(Return(remoteInput));

    EXPECT_NEAR(expectedValue, operatorInterface.getTurretYawInput(1), 1E-3);
}

TEST_P(TurretTest, getTurretPitchInput_returns_user_input)
{
    auto params = GetParam();

    float remoteInput = std::get<0>(params);
    int16_t mouseInput = std::get<1>(params);
    float expectedValue = std::get<2>(params);

    EXPECT_CALL(drivers.remote, getMouseY).WillOnce(Return(-mouseInput));
    EXPECT_CALL(drivers.remote, getChannel(Remote::Channel::RIGHT_VERTICAL))
        .WillOnce(Return(remoteInput));

    EXPECT_NEAR(expectedValue, operatorInterface.getTurretPitchInput(0), 1E-3);
}

TEST_P(TurretTest, getTurretPitchInput_turret2_returns_user_input)
{
    auto params = GetParam();

    float remoteInput = std::get<0>(params);
    int16_t mouseInput = std::get<1>(params);
    float expectedValue = std::get<2>(params);

    EXPECT_CALL(drivers.remote, getMouseY).WillOnce(Return(-mouseInput));
    EXPECT_CALL(drivers.remote, getChannel(Remote::Channel::LEFT_VERTICAL))
        .WillOnce(Return(remoteInput));

    EXPECT_NEAR(expectedValue, operatorInterface.getTurretPitchInput(1), 1E-3);
}

INSTANTIATE_TEST_SUITE_P(
    ControlOperatorInterface,
    TurretTest,
    Values(
        std::tuple<float, int16_t, float>(0, 0, 0),
        std::tuple<float, int16_t, float>(-1, 0, 1),
        std::tuple<float, int16_t, float>(-0.9, 0, 0.9),
        std::tuple<float, int16_t, float>(-0.9, 0, 0.9),
        std::tuple<float, int16_t, float>(-0.6, 0, 0.6),
        std::tuple<float, int16_t, float>(-0.2, 0, 0.2),
        std::tuple<float, int16_t, float>(0, INT16_MIN + 1, 1),
        std::tuple<float, int16_t, float>(0, INT16_MAX - 1, -1),
        std::tuple<float, int16_t, float>(1, INT16_MIN + 1, 0)));

class SentryChassisTest : public ControlOperatorInterfaceTest,
                          public WithParamInterface<std::tuple<float, float>>
{
};

TEST_P(SentryChassisTest, getSentrySpeedInput_retuns_user_input)
{
    ON_CALL(drivers.remote, getChannel(Remote::Channel::WHEEL))
        .WillByDefault(Return(std::get<0>(GetParam())));

    EXPECT_NEAR(std::get<1>(GetParam()), operatorInterface.getSentrySpeedInput(), 1E-3);
}

INSTANTIATE_TEST_SUITE_P(
    ControlOperatorInterface,
    SentryChassisTest,
    Values(
        std::tuple<float, float>(0, 0),
        std::tuple<float, float>(-660, ControlOperatorInterface::USER_STICK_SENTRY_DRIVE_SCALAR),
        std::tuple<float, float>(660, -ControlOperatorInterface::USER_STICK_SENTRY_DRIVE_SCALAR),
        std::tuple<float, float>(
            330,
            -0.5 * ControlOperatorInterface::USER_STICK_SENTRY_DRIVE_SCALAR),
        std::tuple<float, float>(
            -330,
            0.5 * ControlOperatorInterface::USER_STICK_SENTRY_DRIVE_SCALAR)));
