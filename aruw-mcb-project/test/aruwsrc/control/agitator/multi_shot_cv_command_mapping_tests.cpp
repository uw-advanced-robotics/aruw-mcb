/*
 * Copyright (c) 2021-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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
#include "tap/mock/command_mock.hpp"
#include "tap/mock/hold_repeat_command_mapping_mock.hpp"
#include "tap/mock/motor_interface_mock.hpp"
#include "tap/mock/odometry_2d_interface_mock.hpp"

#include "aruwsrc/control/agitator/multi_shot_cv_command_mapping.hpp"
#include "aruwsrc/control/auto-aim/auto_aim_fire_rate_reselection_manager.hpp"
#include "aruwsrc/control/turret/algorithms/chassis_frame_turret_controller.hpp"
#include "aruwsrc/mock/control_operator_interface_mock.hpp"
#include "aruwsrc/mock/cv_on_target_governor_mock.hpp"
#include "aruwsrc/mock/launch_speed_predictor_interface_mock.hpp"
#include "aruwsrc/mock/manual_fire_rate_reselection_manager_mock.hpp"
#include "aruwsrc/mock/otto_ballistics_solver_mock.hpp"
#include "aruwsrc/mock/robot_turret_subsystem_mock.hpp"
#include "aruwsrc/mock/turret_cv_command_mock.hpp"
#include "aruwsrc/mock/turret_motor_mock.hpp"
#include "aruwsrc/mock/vision_coprocessor_mock.hpp"

using namespace testing;
using namespace aruwsrc::control::agitator;

class MultiShotCvCommandMappingTest : public Test
{
protected:
    MultiShotCvCommandMappingTest()
        : drivers(),
          yawMotor(&yawM, {}),
          pitchMotor(&pitM, {}),
          yawController(yawMotor, {}),
          pitchController(pitchMotor, {}),
          turretSubsystem(&drivers),
          visionCoprocessor(&drivers),
          operatorInterface(&drivers),
          ballisticsSolver(visionCoprocessor, odometry, turretSubsystem, launcher, 0, 0),
          turretCvCommand(
              &visionCoprocessor,
              &operatorInterface,
              &turretSubsystem,
              &yawController,
              &pitchController,
              &ballisticsSolver,
              0,
              0),
          launchTimer(0, nullptr, nullptr),
          defaultRms(
              tap::communication::serial::Remote::Switch::LEFT_SWITCH,
              tap::communication::serial::Remote::SwitchState::UP),
          cvOnTargetGovernor(
              &drivers,
              visionCoprocessor,
              turretCvCommand,
              launchTimer,
              aruwsrc::control::governor::CvOnTargetGovernorMode::ON_TARGET_AND_GATED),
          multiShotCommandMapping(drivers, cmd, defaultRms, &fireRateManager, cvOnTargetGovernor)
    {
    }

    void SetUp() override
    {
        ON_CALL(drivers.commandScheduler, isCommandScheduled).WillByDefault(Return(false));
    }

    tap::Drivers drivers;

private:
    NiceMock<tap::mock::MotorInterfaceMock> yawM;
    NiceMock<tap::mock::MotorInterfaceMock> pitM;
    aruwsrc::mock::TurretMotorMock yawMotor;
    aruwsrc::mock::TurretMotorMock pitchMotor;
    aruwsrc::control::turret::algorithms::ChassisFrameYawTurretController yawController;
    aruwsrc::control::turret::algorithms::ChassisFramePitchTurretController pitchController;
    NiceMock<aruwsrc::mock::RobotTurretSubsystemMock> turretSubsystem;
    NiceMock<aruwsrc::mock::VisionCoprocessorMock> visionCoprocessor;
    NiceMock<aruwsrc::mock::ControlOperatorInterfaceMock> operatorInterface;
    NiceMock<aruwsrc::mock::LaunchSpeedPredictorInterfaceMock> launcher;
    NiceMock<tap::mock::Odometry2DInterfaceMock> odometry;
    NiceMock<aruwsrc::mock::OttoBallisticsSolverMock> ballisticsSolver;
    NiceMock<aruwsrc::mock::TurretCVCommandMock> turretCvCommand;

    aruwsrc::control::governor::AutoAimLaunchTimer launchTimer;

protected:
    NiceMock<tap::mock::CommandMock> cmd;
    tap::control::RemoteMapState defaultRms;
    NiceMock<aruwsrc::mock::ManualFireRateReselectionManagerMock> fireRateManager;
    NiceMock<aruwsrc::mock::CvOnTargetGovernorMock> cvOnTargetGovernor;
    MultiShotCvCommandMapping multiShotCommandMapping;
};

TEST_F(MultiShotCvCommandMappingTest, getShooterState_matches_setShooterState)
{
    for (uint8_t i = MultiShotCvCommandMapping::SINGLE;
         i < MultiShotCvCommandMapping::NUM_SHOOTER_STATES;
         i++)
    {
        multiShotCommandMapping.setShooterState(
            static_cast<MultiShotCvCommandMapping::LaunchMode>(i));
        EXPECT_EQ(i, multiShotCommandMapping.getLaunchMode());
    }
}

TEST_F(MultiShotCvCommandMappingTest, setShooterState_single_adds_command_once)
{
    ON_CALL(cvOnTargetGovernor, inShotTimingMode).WillByDefault(Return(false));

    EXPECT_CALL(drivers.commandScheduler, addCommand).Times(1);

    EXPECT_CALL(fireRateManager, setFireRate(ManualFireRateReselectionManager::MAX_FIRERATE_RPS))
        .Times(4);

    multiShotCommandMapping.setShooterState(MultiShotCvCommandMapping::SINGLE);

    multiShotCommandMapping.executeCommandMapping(defaultRms);
    multiShotCommandMapping.executeCommandMapping(defaultRms);
    multiShotCommandMapping.executeCommandMapping(defaultRms);
    multiShotCommandMapping.executeCommandMapping(defaultRms);
}

TEST_F(MultiShotCvCommandMappingTest, setShooterState_10hz_full_repeatedly_adds_commands)
{
    ON_CALL(cvOnTargetGovernor, inShotTimingMode).WillByDefault(Return(false));

    {
        InSequence seq;
        EXPECT_CALL(fireRateManager, setFireRate(10)).Times(4);
        EXPECT_CALL(fireRateManager, setFireRate(20)).Times(4);
        EXPECT_CALL(
            fireRateManager,
            setFireRate(ManualFireRateReselectionManager::MAX_FIRERATE_RPS))
            .Times(4);
    }

    EXPECT_CALL(drivers.commandScheduler, addCommand).Times(12);

    multiShotCommandMapping.setShooterState(MultiShotCvCommandMapping::LIMITED_10HZ);

    multiShotCommandMapping.executeCommandMapping(defaultRms);
    multiShotCommandMapping.executeCommandMapping(defaultRms);
    multiShotCommandMapping.executeCommandMapping(defaultRms);
    multiShotCommandMapping.executeCommandMapping(defaultRms);

    multiShotCommandMapping.setShooterState(MultiShotCvCommandMapping::LIMITED_20HZ);

    multiShotCommandMapping.executeCommandMapping(defaultRms);
    multiShotCommandMapping.executeCommandMapping(defaultRms);
    multiShotCommandMapping.executeCommandMapping(defaultRms);
    multiShotCommandMapping.executeCommandMapping(defaultRms);

    multiShotCommandMapping.setShooterState(MultiShotCvCommandMapping::FULL_AUTO);

    multiShotCommandMapping.executeCommandMapping(defaultRms);
    multiShotCommandMapping.executeCommandMapping(defaultRms);
    multiShotCommandMapping.executeCommandMapping(defaultRms);
    multiShotCommandMapping.executeCommandMapping(defaultRms);
}

TEST_F(
    MultiShotCvCommandMappingTest,
    setShooterState_governor_gating_single_shot_repeatedly_adds_commands)
{
    ON_CALL(cvOnTargetGovernor, inShotTimingMode).WillByDefault(Return(true));

    EXPECT_CALL(drivers.commandScheduler, addCommand).Times(4);

    multiShotCommandMapping.setShooterState(MultiShotCvCommandMapping::SINGLE);

    multiShotCommandMapping.executeCommandMapping(defaultRms);
    multiShotCommandMapping.executeCommandMapping(defaultRms);
    multiShotCommandMapping.executeCommandMapping(defaultRms);
    multiShotCommandMapping.executeCommandMapping(defaultRms);
}

TEST_F(
    MultiShotCvCommandMappingTest,
    executeCommandMapping_fireReselection_manager_missing_doesnt_crash)
{
    ON_CALL(cvOnTargetGovernor, inShotTimingMode).WillByDefault(Return(false));

    MultiShotCvCommandMapping multiShotCommandMapping(
        drivers,
        cmd,
        defaultRms,
        std::nullopt,
        cvOnTargetGovernor);

    multiShotCommandMapping.executeCommandMapping(defaultRms);
}
