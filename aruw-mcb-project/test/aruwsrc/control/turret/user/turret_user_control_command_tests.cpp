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

#include "aruwsrc/control/turret/algorithms/chassis_frame_turret_controller.hpp"
#include "aruwsrc/control/turret/algorithms/turret_gravity_compensation.hpp"
#include "aruwsrc/control/turret/user/turret_user_control_command.hpp"
#include "aruwsrc/drivers.hpp"
#include "aruwsrc/mock/turret_subsystem_mock.hpp"

using namespace aruwsrc;
using namespace aruwsrc::control::turret;
using namespace aruwsrc::control::turret::user;
using namespace aruwsrc::control::turret::algorithms;
using namespace aruwsrc::mock;
using namespace testing;

#define SETUP_TEST()                                                                            \
    Drivers drivers;                                                                            \
    NiceMock<TurretSubsystemMock> turret(&drivers);                                             \
    ChassisFramePitchTurretController pitchController(&turret, {1, 0, 0, 0, 1, 1, 0, 1, 0, 0}); \
    ChassisFrameYawTurretController yawController(&turret, {1, 0, 0, 0, 1, 1, 0, 1, 0, 0});     \
    TurretUserControlCommand turretCmd(&drivers, &turret, &yawController, &pitchController);

TEST(TurretUserControlCommand, isReady_return_true_when_turret_online)
{
    SETUP_TEST();

    ON_CALL(turret, isOnline).WillByDefault(Return(true));

    EXPECT_TRUE(turretCmd.isReady());
}

TEST(TurretUserControlCommand, isReady_return_false_when_turret_offline)
{
    SETUP_TEST();

    ON_CALL(turret, isOnline).WillByDefault(Return(false));

    EXPECT_FALSE(turretCmd.isReady());
}

TEST(TurretUserControlCommand, isFinished_return_true_when_turret_offline)
{
    SETUP_TEST();

    ON_CALL(turret, isOnline).WillByDefault(Return(false));

    EXPECT_TRUE(turretCmd.isFinished());
}

TEST(TurretUserControlCommand, isFinished_return_false_when_turret_online)
{
    SETUP_TEST();

    ON_CALL(turret, isOnline).WillByDefault(Return(true));

    EXPECT_FALSE(turretCmd.isFinished());
}

TEST(TurretUserControlCommand, end_sets_motor_out_to_0)
{
    SETUP_TEST();

    EXPECT_CALL(turret, setPitchMotorOutput(0)).Times(2);
    EXPECT_CALL(turret, setYawMotorOutput(0)).Times(2);

    turretCmd.end(true);
    turretCmd.end(false);
}

TEST(TurretUserControlCommand, execute_output_0_when_error_0)
{
    SETUP_TEST();

    tap::algorithms::ContiguousFloat yawActual(90, 0, 360);
    tap::algorithms::ContiguousFloat pitchActual(90, 0, 360);
    ON_CALL(drivers.controlOperatorInterface, getTurretPitchInput).WillByDefault(Return(0));
    ON_CALL(drivers.controlOperatorInterface, getTurretYawInput).WillByDefault(Return(0));
    ON_CALL(turret, getPitchSetpoint).WillByDefault(Return(90));
    ON_CALL(turret, getYawSetpoint).WillByDefault(Return(90));
    ON_CALL(turret, getCurrentYawValue).WillByDefault(ReturnRef(yawActual));
    ON_CALL(turret, getCurrentPitchValue).WillByDefault(ReturnRef(pitchActual));
    ON_CALL(turret, getPitchVelocity).WillByDefault(Return(0));
    ON_CALL(turret, getYawVelocity).WillByDefault(Return(0));

    EXPECT_CALL(
        turret,
        setPitchMotorOutput(FloatNear(computeGravitationalForceOffset(
            TurretSubsystem::TURRET_CG_X,
            TurretSubsystem::TURRET_CG_Z,
            0,
            TurretSubsystem::GRAVITY_COMPENSATION_SCALAR), 1E-3)));
    EXPECT_CALL(turret, setYawMotorOutput(0));
    EXPECT_CALL(turret, setPitchSetpoint(90));
    EXPECT_CALL(turret, setYawSetpoint(90));

    turretCmd.initialize();
    turretCmd.execute();
}

TEST(TurretUserControlCommand, execute_output_nonzero_when_error_nonzero)
{
    SETUP_TEST();

    float pitchSetpoint = 90, yawSetpoint = 90;
    tap::algorithms::ContiguousFloat yawActual(90, 0, 360);
    tap::algorithms::ContiguousFloat pitchActual(90, 0, 360);
    ON_CALL(drivers.controlOperatorInterface, getTurretPitchInput).WillByDefault(Return(1));
    ON_CALL(drivers.controlOperatorInterface, getTurretYawInput).WillByDefault(Return(-1));
    ON_CALL(turret, getPitchSetpoint).WillByDefault(ReturnPointee(&pitchSetpoint));
    ON_CALL(turret, getYawSetpoint).WillByDefault(ReturnPointee(&yawSetpoint));
    ON_CALL(turret, getCurrentYawValue).WillByDefault(ReturnRef(yawActual));
    ON_CALL(turret, getCurrentPitchValue).WillByDefault(ReturnRef(pitchActual));
    ON_CALL(turret, getPitchVelocity).WillByDefault(Return(0));
    ON_CALL(turret, getYawVelocity).WillByDefault(Return(0));

    EXPECT_CALL(
        turret,
        setPitchMotorOutput(Gt(computeGravitationalForceOffset(
            TurretSubsystem::TURRET_CG_X,
            TurretSubsystem::TURRET_CG_Z,
            0,
            TurretSubsystem::GRAVITY_COMPENSATION_SCALAR))));
    EXPECT_CALL(turret, setYawMotorOutput(Lt(0)));
    EXPECT_CALL(turret, setPitchSetpoint(Gt(90))).WillRepeatedly([&](float setpoint) {
        pitchSetpoint = setpoint;
    });
    EXPECT_CALL(turret, setYawSetpoint(Lt(90))).WillRepeatedly([&](float setpoint) {
        yawSetpoint = setpoint;
    });

    turretCmd.initialize();
    turretCmd.execute();
}
