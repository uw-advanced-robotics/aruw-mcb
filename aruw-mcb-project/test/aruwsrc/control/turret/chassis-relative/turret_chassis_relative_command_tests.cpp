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

#include "aruwsrc/control/turret/algorithms/turret_gravity_compensation.hpp"
#include "aruwsrc/control/turret/chassis-relative/turret_chassis_relative_command.hpp"
#include "aruwsrc/mock/turret_subsystem_mock.hpp"

using namespace tap;
using namespace aruwsrc::control::turret;
using namespace aruwsrc::mock;
using namespace testing;

TEST(TurretChassisRelativeCommand, isReady__return_true_when_turret_online)
{
    Drivers drivers;
    TurretSubsystemMock turret(&drivers);
    TurretChassisRelativeCommand turretCR(&drivers, &turret);

    EXPECT_CALL(turret, isOnline).WillOnce(Return(true)).WillOnce(Return(false));

    EXPECT_TRUE(turretCR.isReady());
    EXPECT_FALSE(turretCR.isReady());
}

TEST(TurretChassisRelativeCommand, isFinished__return_true_when_turret_offline)
{
    Drivers drivers;
    TurretSubsystemMock turret(&drivers);
    TurretChassisRelativeCommand turretCR(&drivers, &turret);

    EXPECT_CALL(turret, isOnline).WillOnce(Return(false)).WillOnce(Return(true));

    EXPECT_TRUE(turretCR.isFinished());
    EXPECT_FALSE(turretCR.isFinished());
}

TEST(TurretChassisRelativeCommand, end__sets_motor_out_to_0)
{
    Drivers drivers;
    TurretSubsystemMock turret(&drivers);
    TurretChassisRelativeCommand turretCR(&drivers, &turret);

    EXPECT_CALL(turret, setPitchMotorOutput(0)).Times(2);
    EXPECT_CALL(turret, setYawMotorOutput(0)).Times(2);

    turretCR.end(true);
    turretCR.end(false);
}

TEST(TurretChassisRelativeCommand, execute__output_0_when_error_0)
{
    Drivers drivers;
    NiceMock<TurretSubsystemMock> turret(&drivers);
    TurretChassisRelativeCommand turretCR(&drivers, &turret);

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
        setPitchMotorOutput(FloatEq(aruwsrc::control::turret::computeGravitationalForceOffset(
            TurretSubsystem::TURRET_CG_X,
            TurretSubsystem::TURRET_CG_Z,
            0,
            TurretSubsystem::GRAVITY_COMPENSATION_SCALAR))));
    EXPECT_CALL(turret, setYawMotorOutput(0));
    EXPECT_CALL(turret, setPitchSetpoint(90));
    EXPECT_CALL(turret, setYawSetpoint(90));

    turretCR.initialize();
    turretCR.execute();
}

TEST(TurretChassisRelativeCommand, execute__output_nonzero_when_error_nonzero)
{
    Drivers drivers;
    NiceMock<TurretSubsystemMock> turret(&drivers);
    TurretChassisRelativeCommand turretCR(&drivers, &turret);

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
        setPitchMotorOutput(Gt(aruwsrc::control::turret::computeGravitationalForceOffset(
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

    turretCR.initialize();
    turretCR.execute();
}
