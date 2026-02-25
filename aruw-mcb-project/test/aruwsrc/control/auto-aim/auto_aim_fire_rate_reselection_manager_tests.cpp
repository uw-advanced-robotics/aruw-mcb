/*
 * Copyright (c) 2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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
#include "tap/mock/motor_interface_mock.hpp"
#include "tap/mock/odometry_2d_interface_mock.hpp"

#include "aruwsrc/control/auto-aim/auto_aim_fire_rate_reselection_manager.hpp"
#include "aruwsrc/control/turret/algorithms/chassis_frame_turret_controller.hpp"
#include "aruwsrc/mock/control_operator_interface_mock.hpp"
#include "aruwsrc/mock/launch_speed_predictor_interface_mock.hpp"
#include "aruwsrc/mock/otto_ballistics_solver_mock.hpp"
#include "aruwsrc/mock/robot_turret_subsystem_mock.hpp"
#include "aruwsrc/mock/turret_cv_command_mock.hpp"
#include "aruwsrc/mock/turret_motor_mock.hpp"
#include "aruwsrc/mock/vision_coprocessor_mock.hpp"

using namespace testing;
using namespace aruwsrc::control::auto_aim;
using namespace aruwsrc::communication::serial;
using namespace aruwsrc::control::agitator;

class AutoAimFireRateManagerTest : public Test
{
protected:
    AutoAimFireRateManagerTest()
        : yawM(),
          pitM(),
          yawMotor(&yawM, {}),
          pitchMotor(&pitM, {}),
          yawController(yawMotor, {}),
          pitchController(pitchMotor, {}),
          turretSubsystem(&drivers),
          visionCoprocessor(&drivers),
          ballisticsSolver(visionCoprocessor, odometry, turretSubsystem, launcher, 0, 0),
          operatorInterface(&drivers),
          turretCvCommand(
              &visionCoprocessor,
              &operatorInterface,
              &turretSubsystem,
              &yawController,
              &pitchController,
              &ballisticsSolver,
              0,
              0),
          fireRateManager(drivers, visionCoprocessor, drivers.commandScheduler, turretCvCommand, 0)
    {
    }

    void SetUp() override
    {
        aimData.pva.updated = true;
        ON_CALL(visionCoprocessor, getLastAimData(0)).WillByDefault(ReturnRef(aimData));
    }

private:
    NiceMock<tap::mock::MotorInterfaceMock> yawM;
    NiceMock<tap::mock::MotorInterfaceMock> pitM;
    aruwsrc::mock::TurretMotorMock yawMotor;
    aruwsrc::mock::TurretMotorMock pitchMotor;
    aruwsrc::control::turret::algorithms::ChassisFrameTurretController<
        aruwsrc::control::turret::algorithms::Axis::YAW>
        yawController;
    aruwsrc::control::turret::algorithms::ChassisFrameTurretController<
        aruwsrc::control::turret::algorithms::Axis::PITCH>
        pitchController;
    NiceMock<aruwsrc::mock::RobotTurretSubsystemMock> turretSubsystem;
    NiceMock<aruwsrc::mock::LaunchSpeedPredictorInterfaceMock> launcher;
    NiceMock<tap::mock::Odometry2DInterfaceMock> odometry;

protected:
    NiceMock<aruwsrc::mock::VisionCoprocessorMock> visionCoprocessor;

private:
    NiceMock<aruwsrc::mock::OttoBallisticsSolverMock> ballisticsSolver;

protected:
    tap::Drivers drivers;
    NiceMock<aruwsrc::mock::ControlOperatorInterfaceMock> operatorInterface;
    NiceMock<aruwsrc::mock::TurretCVCommandMock> turretCvCommand;
    AutoAimFireRateReselectionManager fireRateManager;
    VisionCoprocessor::TurretAimData aimData = {};
};

TEST_F(AutoAimFireRateManagerTest, getFireRateReadinessState_ignore_limiting_CV_not_running)
{
    ON_CALL(drivers.commandScheduler, isCommandScheduled(&turretCvCommand))
        .WillByDefault(Return(false));

    EXPECT_EQ(
        FireRateReadinessState::READY_IGNORE_RATE_LIMITING,
        fireRateManager.getFireRateReadinessState());
}

TEST_F(AutoAimFireRateManagerTest, getFireRateReadinessState_not_ready_cv_running_but_cv_not_online)
{
    ON_CALL(drivers.commandScheduler, isCommandScheduled(&turretCvCommand))
        .WillByDefault(Return(true));
    ON_CALL(visionCoprocessor, isCvOnline).WillByDefault(Return(false));

    EXPECT_EQ(FireRateReadinessState::NOT_READY, fireRateManager.getFireRateReadinessState());
}

TEST_F(AutoAimFireRateManagerTest, getFireRateReadinessState_not_ready_zero_firerate)
{
    ON_CALL(drivers.commandScheduler, isCommandScheduled(&turretCvCommand))
        .WillByDefault(Return(true));
    ON_CALL(visionCoprocessor, isCvOnline).WillByDefault(Return(true));

#ifdef USE_VISION_COPROCESSOR_SENT_FIRE_RATE
    aimData.pva.firerate = VisionCoprocessor::FireRate::ZERO;
#else
    aimData.pva.updated = false;
#endif

    EXPECT_EQ(FireRateReadinessState::NOT_READY, fireRateManager.getFireRateReadinessState());
}

TEST_F(AutoAimFireRateManagerTest, getFireRateReadinessState_ready_nonzero_firerate)
{
    ON_CALL(drivers.commandScheduler, isCommandScheduled(&turretCvCommand))
        .WillByDefault(Return(true));
    ON_CALL(visionCoprocessor, isCvOnline).WillByDefault(Return(true));

    aimData.pva.firerate = VisionCoprocessor::FireRate::LOW;
    EXPECT_EQ(
        FireRateReadinessState::READY_USE_RATE_LIMITING,
        fireRateManager.getFireRateReadinessState());
    aimData.pva.firerate = VisionCoprocessor::FireRate::MEDIUM;
    EXPECT_EQ(
        FireRateReadinessState::READY_USE_RATE_LIMITING,
        fireRateManager.getFireRateReadinessState());
    aimData.pva.firerate = VisionCoprocessor::FireRate::HIGH;
    EXPECT_EQ(
        FireRateReadinessState::READY_USE_RATE_LIMITING,
        fireRateManager.getFireRateReadinessState());
}

using TestParams = std::tuple<uint32_t, VisionCoprocessor::FireRate, float>;

class AutoAimFireRateManagerTestParameterized : public AutoAimFireRateManagerTest,
                                                public WithParamInterface<TestParams>
{
    void SetUp() override
    {
        aimData.pva.firerate = std::get<1>(GetParam());
        aimData.pva.xPos = std::get<2>(GetParam());
        aimData.pva.yPos = 0;
        aimData.pva.zPos = 0;
        AutoAimFireRateManagerTest::SetUp();
    }
};

TEST_P(AutoAimFireRateManagerTestParameterized, getFireRatePeriod)
{
    EXPECT_EQ(std::get<0>(GetParam()), fireRateManager.getFireRatePeriod());
}

#ifdef USE_VISION_COPROCESSOR_SENT_FIRE_RATE
static constexpr TestParams TEST_ZERO_FIRERATE{
    0,
    VisionCoprocessor::FireRate::ZERO,
    0.0f};

static TestParams TEST_LOW_FIRERATE{
    uint32_t(round(1000.0f / AutoAimFireRateReselectionManager::LOW_RPS)),
    VisionCoprocessor::FireRate::LOW,
    1.0f};

static TestParams TEST_MID_FIRERATE{
    uint32_t(round(1000.0f / AutoAimFireRateReselectionManager::MID_RPS)),
    VisionCoprocessor::FireRate::MEDIUM,
    6.0f};

static TestParams TEST_HIGH_FIRERATE{
    uint32_t(round(1000.0f / AutoAimFireRateReselectionManager::HIGH_RPS)),
    VisionCoprocessor::FireRate::HIGH,
    12.0f};
#else
// Range-based mode ignores the vision firerate enum and derives fire rate from target range.
static TestParams TEST_ZERO_FIRERATE{
    uint32_t(round(1000.0f / AutoAimFireRateReselectionManager::HIGH_RPS)),
    VisionCoprocessor::FireRate::ZERO,
    1.0f};

static TestParams TEST_LOW_FIRERATE{
    uint32_t(round(1000.0f / AutoAimFireRateReselectionManager::HIGH_RPS)),
    VisionCoprocessor::FireRate::LOW,
    1.0f};

static TestParams TEST_MID_FIRERATE{
    uint32_t(round(1000.0f / AutoAimFireRateReselectionManager::MID_RPS)),
    VisionCoprocessor::FireRate::MEDIUM,
    6.0f};

static TestParams TEST_HIGH_FIRERATE{
    uint32_t(round(1000.0f / AutoAimFireRateReselectionManager::LOW_RPS)),
    VisionCoprocessor::FireRate::HIGH,
    12.0f};
#endif

INSTANTIATE_TEST_CASE_P(
    AutoAimFireRateManagerTest,
    AutoAimFireRateManagerTestParameterized,
    Values(TEST_ZERO_FIRERATE, TEST_LOW_FIRERATE, TEST_MID_FIRERATE, TEST_HIGH_FIRERATE));
