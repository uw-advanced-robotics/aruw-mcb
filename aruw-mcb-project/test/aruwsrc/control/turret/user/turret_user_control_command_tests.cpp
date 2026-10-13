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

#include "aruwsrc/control/turret/algorithms/chassis_frame_turret_controller.hpp"
#include "aruwsrc/control/turret/algorithms/turret_gravity_compensation.hpp"
#include "aruwsrc/control/turret/constants/turret_constants.hpp"
#include "aruwsrc/control/turret/user/turret_user_control_command.hpp"
#include "aruwsrc/mock/control_operator_interface_mock.hpp"
#include "aruwsrc/mock/turret_subsystem_mock.hpp"

using namespace aruwsrc;
using namespace aruwsrc::control::turret;
using namespace aruwsrc::control::turret::user;
using namespace aruwsrc::control::turret::algorithms;
using namespace aruwsrc::mock;
using namespace testing;

using tap::algorithms::transforms::Axis;

#define SETUP_TEST()

static constexpr aruwsrc::control::turret::algorithms::TurretGravitationalForceOffset::
    TurretGravityParams TURRET_GRAVITY_CONFIG_TEST{
        .cgX = 0.1f,
        .cgZ = 0.1f,
        .gravityCompensatorMax = 1000.0f};
namespace
{
float computeGravitationalForceOffset(const float pitchAngleRad)
{
    TurretGravitationalForceOffset gravityCompensation(TURRET_GRAVITY_CONFIG_TEST);
    return gravityCompensation.calculateCompensationEffort(
        {.pitchWorldFrame = pitchAngleRad, .yaw = 0.0f});
};
TurretGravitationalForceOffset gravityCompensation(TURRET_GRAVITY_CONFIG_TEST);
}  // namespace

class TurretUserControlCommandTest : public Test
{
protected:
    TurretUserControlCommandTest()
        : pitchMotorMock(&pitchMotorInterface),
          yawMotorMock(&yawMotorInterface),
          turret(&drivers, pitchMotorMock, yawMotorMock, nullptr),
          controlOperatorInterface(&drivers),
          pitchController(pitchMotorMock, {1, 0, 0, 0, 1, 1, 0, 1, 0, 0}, {&gravityCompensation}),
          yawController(yawMotorMock, {1, 0, 0, 0, 1, 1, 0, 1, 0, 0}),
          turretCmd(
              &drivers,
              controlOperatorInterface,
              &turret,
              &yawController,
              &pitchController,
              1.0f,
              1.0f)
    {
    }

    tap::Drivers drivers;
    NiceMock<tap::mock::MotorInterfaceMock> pitchMotorInterface;
    NiceMock<tap::mock::MotorInterfaceMock> yawMotorInterface;
    NiceMock<aruwsrc::mock::TurretMotorMock> pitchMotorMock;
    NiceMock<aruwsrc::mock::TurretMotorMock> yawMotorMock;
    NiceMock<TurretSubsystemMock> turret;
    NiceMock<ControlOperatorInterfaceMock> controlOperatorInterface;
    ChassisFrameTurretController<Axis::PITCH> pitchController;
    ChassisFrameTurretController<Axis::YAW> yawController;
    TurretUserControlCommand turretCmd;
};

TEST_F(TurretUserControlCommandTest, isReady_return_true_when_turret_online)
{
    ON_CALL(yawMotorMock, isOnline).WillByDefault(Return(true));
    ON_CALL(pitchMotorMock, isOnline).WillByDefault(Return(true));

    EXPECT_TRUE(turretCmd.isReady());
}

TEST_F(TurretUserControlCommandTest, isReady_return_false_when_turret_offline)
{
    ON_CALL(yawMotorMock, isOnline).WillByDefault(Return(false));
    ON_CALL(pitchMotorMock, isOnline).WillByDefault(Return(false));

    EXPECT_FALSE(turretCmd.isReady());
}

TEST_F(TurretUserControlCommandTest, isFinished_return_true_when_turret_offline)
{
    ON_CALL(yawMotorMock, isOnline).WillByDefault(Return(false));
    ON_CALL(pitchMotorMock, isOnline).WillByDefault(Return(false));

    EXPECT_TRUE(turretCmd.isFinished());
}

TEST_F(TurretUserControlCommandTest, isFinished_return_false_when_turret_online)
{
    ON_CALL(yawMotorMock, isOnline).WillByDefault(Return(true));
    ON_CALL(pitchMotorMock, isOnline).WillByDefault(Return(true));

    EXPECT_FALSE(turretCmd.isFinished());
}

TEST_F(TurretUserControlCommandTest, end_sets_motor_out_to_0)
{
    EXPECT_CALL(yawMotorMock, setMotorOutput(0)).Times(2);
    EXPECT_CALL(pitchMotorMock, setMotorOutput(0)).Times(2);

    turretCmd.end(true);
    turretCmd.end(false);
}

TEST_F(TurretUserControlCommandTest, execute_output_0_when_error_0)
{
    WrappedFloat yawActual = tap::algorithms::Angle(M_PI_2);
    WrappedFloat pitchActual = tap::algorithms::Angle(M_PI_2);
    WrappedFloat yawSetpoint = tap::algorithms::Angle(M_PI_2);
    WrappedFloat pitchSetpoint = tap::algorithms::Angle(M_PI_2);

    ON_CALL(controlOperatorInterface, getTurretPitchInput).WillByDefault(Return(0));
    ON_CALL(controlOperatorInterface, getTurretYawInput).WillByDefault(Return(0));
    ON_CALL(pitchMotorMock, getChassisFrameSetpoint).WillByDefault(ReturnPointee(&yawSetpoint));
    ON_CALL(yawMotorMock, getChassisFrameSetpoint).WillByDefault(ReturnPointee(&pitchSetpoint));
    ON_CALL(pitchMotorMock, getChassisFrameMeasuredAngle).WillByDefault(ReturnRef(pitchActual));
    ON_CALL(yawMotorMock, getChassisFrameMeasuredAngle).WillByDefault(ReturnRef(yawActual));
    ON_CALL(pitchMotorMock, getChassisFrameVelocity).WillByDefault(Return(0));
    ON_CALL(yawMotorMock, getChassisFrameVelocity).WillByDefault(Return(0));

    EXPECT_CALL(
        pitchMotorMock,
        setMotorOutput(
            FloatNear(computeGravitationalForceOffset(pitchActual.getWrappedValue()), 1E-2)));
    EXPECT_CALL(yawMotorMock, setMotorOutput(0));
    EXPECT_CALL(
        pitchMotorMock,
        setChassisFrameSetpoint(Property(&WrappedFloat::getWrappedValue, M_PI_2)));
    EXPECT_CALL(
        yawMotorMock,
        setChassisFrameSetpoint(Property(&WrappedFloat::getWrappedValue, M_PI_2)));

    turretCmd.initialize();
    turretCmd.execute();
}

TEST_F(TurretUserControlCommandTest, execute_output_nonzero_when_error_nonzero)
{
    WrappedFloat pitchSetpoint = tap::algorithms::Angle(M_PI_2);
    WrappedFloat yawSetpoint = tap::algorithms::Angle(M_PI_2);
    WrappedFloat yawActual = tap::algorithms::Angle(M_PI_2);
    WrappedFloat pitchActual = tap::algorithms::Angle(M_PI_2);
    ON_CALL(controlOperatorInterface, getTurretPitchInput).WillByDefault(Return(1));
    ON_CALL(controlOperatorInterface, getTurretYawInput).WillByDefault(Return(-1));
    ON_CALL(pitchMotorMock, getChassisFrameSetpoint).WillByDefault(ReturnPointee(&pitchSetpoint));
    ON_CALL(yawMotorMock, getChassisFrameSetpoint).WillByDefault(ReturnPointee(&yawSetpoint));
    ON_CALL(yawMotorMock, getChassisFrameMeasuredAngle).WillByDefault(ReturnRef(yawActual));
    ON_CALL(pitchMotorMock, getChassisFrameMeasuredAngle).WillByDefault(ReturnRef(pitchActual));
    ON_CALL(pitchMotorMock, getChassisFrameVelocity).WillByDefault(Return(0));
    ON_CALL(yawMotorMock, getChassisFrameVelocity).WillByDefault(Return(0));

    EXPECT_CALL(
        pitchMotorMock,
        setMotorOutput(Gt(computeGravitationalForceOffset(pitchActual.getWrappedValue()))));
    EXPECT_CALL(yawMotorMock, setMotorOutput(Lt(0)));
    EXPECT_CALL(
        pitchMotorMock,
        setChassisFrameSetpoint(
            Matcher<WrappedFloat>(Property(&WrappedFloat::getUnwrappedValue, Gt(M_PI_2)))))
        .WillRepeatedly([&](WrappedFloat setpoint) { pitchSetpoint = setpoint; });
    EXPECT_CALL(
        yawMotorMock,
        setChassisFrameSetpoint(
            Matcher<WrappedFloat>(Property(&WrappedFloat::getUnwrappedValue, Lt(M_PI_2)))))
        .WillRepeatedly([&](WrappedFloat setpoint) { yawSetpoint = setpoint; });

    turretCmd.initialize();
    turretCmd.execute();
}
