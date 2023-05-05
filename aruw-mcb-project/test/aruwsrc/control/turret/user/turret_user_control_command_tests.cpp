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

#define SETUP_TEST()

class TurretUserControlCommandTest : public Test
{
protected:
    TurretUserControlCommandTest()
        : turret(&drivers),
          controlOperatorInterface(&drivers),
          pitchController(turret.pitchMotor, {1, 0, 0, 0, 1, 1, 0, 1, 0, 0}),
          yawController(turret.yawMotor, {1, 0, 0, 0, 1, 1, 0, 1, 0, 0}),
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
    NiceMock<TurretSubsystemMock> turret;
    NiceMock<ControlOperatorInterfaceMock> controlOperatorInterface;
    ChassisFramePitchTurretController pitchController;
    ChassisFrameYawTurretController yawController;
    TurretUserControlCommand turretCmd;
};

TEST_F(TurretUserControlCommandTest, isReady_return_true_when_turret_online)
{
    ON_CALL(turret.yawMotor, isOnline).WillByDefault(Return(true));
    ON_CALL(turret.pitchMotor, isOnline).WillByDefault(Return(true));

    EXPECT_TRUE(turretCmd.isReady());
}

TEST_F(TurretUserControlCommandTest, isReady_return_false_when_turret_offline)
{
    ON_CALL(turret.yawMotor, isOnline).WillByDefault(Return(false));
    ON_CALL(turret.pitchMotor, isOnline).WillByDefault(Return(false));

    EXPECT_FALSE(turretCmd.isReady());
}

TEST_F(TurretUserControlCommandTest, isFinished_return_true_when_turret_offline)
{
    ON_CALL(turret.yawMotor, isOnline).WillByDefault(Return(false));
    ON_CALL(turret.pitchMotor, isOnline).WillByDefault(Return(false));

    EXPECT_TRUE(turretCmd.isFinished());
}

TEST_F(TurretUserControlCommandTest, isFinished_return_false_when_turret_online)
{
    ON_CALL(turret.yawMotor, isOnline).WillByDefault(Return(true));
    ON_CALL(turret.pitchMotor, isOnline).WillByDefault(Return(true));

    EXPECT_FALSE(turretCmd.isFinished());
}

TEST_F(TurretUserControlCommandTest, end_sets_motor_out_to_0)
{
    EXPECT_CALL(turret.yawMotor, setMotorOutput(0)).Times(2);
    EXPECT_CALL(turret.pitchMotor, setMotorOutput(0)).Times(2);

    turretCmd.end(true);
    turretCmd.end(false);
}

TEST_F(TurretUserControlCommandTest, execute_output_0_when_error_0)
{
    tap::algorithms::WrappedFloat yawActual(M_PI_2, 0, M_TWOPI);
    tap::algorithms::WrappedFloat pitchActual(M_PI_2, 0, M_TWOPI);
    float yawSetpoint = M_PI_2;
    float pitchSetpoint = M_PI_2;

    ON_CALL(controlOperatorInterface, getTurretPitchInput).WillByDefault(Return(0));
    ON_CALL(controlOperatorInterface, getTurretYawInput).WillByDefault(Return(0));
    ON_CALL(turret.pitchMotor, getChassisFrameSetpoint).WillByDefault(ReturnPointee(&yawSetpoint));
    ON_CALL(turret.yawMotor, getChassisFrameSetpoint).WillByDefault(ReturnPointee(&pitchSetpoint));
    ON_CALL(turret.pitchMotor, getChassisFrameMeasuredAngle).WillByDefault(ReturnRef(pitchActual));
    ON_CALL(turret.yawMotor, getChassisFrameMeasuredAngle).WillByDefault(ReturnRef(yawActual));
    ON_CALL(turret.pitchMotor, getChassisFrameVelocity).WillByDefault(Return(0));
    ON_CALL(turret.yawMotor, getChassisFrameVelocity).WillByDefault(Return(0));

    EXPECT_CALL(
        turret.pitchMotor,
        setMotorOutput(FloatNear(
            computeGravitationalForceOffset(
                TURRET_CG_X,
                TURRET_CG_Z,
                0,
                GRAVITY_COMPENSATION_SCALAR),
            1E-3)));
    EXPECT_CALL(turret.yawMotor, setMotorOutput(0));
    EXPECT_CALL(turret.pitchMotor, setChassisFrameSetpoint(M_PI_2));
    EXPECT_CALL(turret.yawMotor, setChassisFrameSetpoint(M_PI_2));

    turretCmd.initialize();
    turretCmd.execute();
}

TEST_F(TurretUserControlCommandTest, execute_output_nonzero_when_error_nonzero)
{
    float pitchSetpoint = M_PI_2;
    float yawSetpoint = M_PI_2;
    tap::algorithms::WrappedFloat yawActual(M_PI_2, 0, M_TWOPI);
    tap::algorithms::WrappedFloat pitchActual(M_PI_2, 0, M_TWOPI);
    ON_CALL(controlOperatorInterface, getTurretPitchInput).WillByDefault(Return(1));
    ON_CALL(controlOperatorInterface, getTurretYawInput).WillByDefault(Return(-1));
    ON_CALL(turret.pitchMotor, getChassisFrameSetpoint)
        .WillByDefault(ReturnPointee(&pitchSetpoint));
    ON_CALL(turret.yawMotor, getChassisFrameSetpoint).WillByDefault(ReturnPointee(&yawSetpoint));
    ON_CALL(turret.yawMotor, getChassisFrameMeasuredAngle).WillByDefault(ReturnRef(yawActual));
    ON_CALL(turret.pitchMotor, getChassisFrameMeasuredAngle).WillByDefault(ReturnRef(pitchActual));
    ON_CALL(turret.pitchMotor, getChassisFrameVelocity).WillByDefault(Return(0));
    ON_CALL(turret.yawMotor, getChassisFrameVelocity).WillByDefault(Return(0));

    EXPECT_CALL(
        turret.pitchMotor,
        setMotorOutput(Gt(computeGravitationalForceOffset(
            TURRET_CG_X,
            TURRET_CG_Z,
            0,
            GRAVITY_COMPENSATION_SCALAR))));
    EXPECT_CALL(turret.yawMotor, setMotorOutput(Lt(0)));
    EXPECT_CALL(turret.pitchMotor, setChassisFrameSetpoint(Gt(M_PI_2)))
        .WillRepeatedly([&](float setpoint) { pitchSetpoint = setpoint; });
    EXPECT_CALL(turret.yawMotor, setChassisFrameSetpoint(Lt(M_PI_2)))
        .WillRepeatedly([&](float setpoint) { yawSetpoint = setpoint; });

    turretCmd.initialize();
    turretCmd.execute();
}
