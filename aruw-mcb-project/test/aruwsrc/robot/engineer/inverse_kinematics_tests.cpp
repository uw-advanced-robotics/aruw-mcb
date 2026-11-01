/*
 * Copyright (c) 2026-2026 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/communication/sensors/current/analog_current_sensor.hpp"
#include "tap/drivers.hpp"
#include "tap/mock/abstract_imu_mock.hpp"
#include "tap/mock/encoder_interface_mock.hpp"
#include "tap/mock/motor_interface_mock.hpp"
#include "tap/mock/odometry_2d_interface_mock.hpp"

#include "aruwsrc/control/turret/algorithms/chassis_frame_turret_controller.hpp"
#include "aruwsrc/robot/engineer/algorithms/engineer_transforms.hpp"
#include "aruwsrc/robot/engineer/algorithms/inverse_kinematics/trajectory_6d.hpp"
#include "aruwsrc/robot/engineer/algorithms/inverse_kinematics/trajectory_ik_command.hpp"

using namespace testing;
using namespace tap::algorithms::transforms;
using namespace aruwsrc::engineer::algorithms::inverse_kinematics;

const float EPS = 1E-5;

inline void expectEq(const Position& actual, const Position& expected, const float epsilon = EPS)
{
    EXPECT_NEAR(actual.x(), expected.x(), epsilon);
    EXPECT_NEAR(actual.y(), expected.y(), epsilon);
    EXPECT_NEAR(actual.z(), expected.z(), epsilon);
}

inline void expectEq(const Vector& actual, const Vector& expected, const float epsilon = EPS)
{
    EXPECT_NEAR(actual.x(), expected.x(), epsilon);
    EXPECT_NEAR(actual.y(), expected.y(), epsilon);
    EXPECT_NEAR(actual.z(), expected.z(), epsilon);
}

inline void expectEq(
    const DynamicPosition& actual,
    const DynamicPosition& expected,
    const float epsilon = EPS)
{
    expectEq(actual.getPosition(), expected.getPosition(), epsilon);
    expectEq(actual.getVelocity(), expected.getVelocity(), epsilon);
    expectEq(actual.getAcceleration(), expected.getAcceleration(), epsilon);
}

inline void expectEq(
    const Orientation& actual,
    const Orientation& expected,
    const float epsilon = EPS)
{
    EXPECT_NEAR(actual.roll(), expected.roll(), epsilon);
    EXPECT_NEAR(actual.pitch(), expected.pitch(), epsilon);
    EXPECT_NEAR(actual.yaw(), expected.yaw(), epsilon);
}

inline void expectEq(
    const AngularVelocity& actual,
    const AngularVelocity& expected,
    const float epsilon = EPS)
{
    EXPECT_NEAR(actual.getRollVelocity(), expected.getRollVelocity(), epsilon);
    EXPECT_NEAR(actual.getPitchVelocity(), expected.getPitchVelocity(), epsilon);
    EXPECT_NEAR(actual.getYawVelocity(), expected.getYawVelocity(), epsilon);
}

inline void expectEq(
    const DynamicOrientation& actual,
    const DynamicOrientation& expected,
    const float epsilon = EPS)
{
    expectEq(actual.getRotation(), expected.getRotation(), epsilon);
    expectEq(actual.getAngularVelocity(), expected.getAngularVelocity(), epsilon);
}

inline void expectStaticEq(
    const Transform& actual,
    const Transform& expected,
    const float epsilon = EPS)
{
    expectEq(actual.getTranslation(), expected.getTranslation(), epsilon);
    expectEq(actual.getRotation(), expected.getRotation(), epsilon);
}

inline void expectEq(const Transform& actual, const Transform& expected, const float epsilon = EPS)
{
    expectEq(actual.getDynamicTranslation(), expected.getDynamicTranslation(), epsilon);
    expectEq(actual.getDynamicOrientation(), expected.getDynamicOrientation(), epsilon);
}

TEST(Trajectory6d, two_waypoint_identical)
{
    Trajectory6D<2> t{
        {{{
              .pose = tap::algorithms::transforms::Transform(),
              .time = 0.0f,
          },
          {
              .pose = tap::algorithms::transforms::Transform(),
              .time = 1.0f,
          }}}};

    for (float i = 0; i <= 1; i += 0.1)
    {
        Transform expected = tap::algorithms::transforms::Transform();
        expectEq(t.atTime(i), expected);
    }
}

TEST(Trajectory6d, two_waypoint_translation_only)
{
    Trajectory6D<2> t{
        {{{
              .pose = tap::algorithms::transforms::Transform(),
              .time = 0.0f,
          },
          {
              .pose = tap::algorithms::transforms::Transform(1, 1, 1, 0, 0, 0),
              .time = 1.0f,
          }}}};

    for (float i = 0; i <= 1; i += 0.1)
    {
        Transform expected = tap::algorithms::transforms::Transform(i, i, i, 0, 0, 0);
        expectEq(t.atTime(i), expected);
    }
}

TEST(Trajectory6d, two_waypoint_yaw_only)
{
    Trajectory6D<2> t{
        {{{
              .pose = tap::algorithms::transforms::Transform(),
              .time = 0.0f,
          },
          {
              .pose = tap::algorithms::transforms::Transform(0, 0, 0, 0, 0, 3),
              .time = 1.0f,
          }}}};

    for (float i = 0; i <= 1; i += 0.1)
    {
        Transform expected = tap::algorithms::transforms::Transform(0, 0, 0, 0, 0, i * 3);
        expectEq(t.atTime(i), expected);
    }
}

TEST(Trajectory6d, two_waypoint_pitch_only)
{
    Trajectory6D<2> t{
        {{{
              .pose = tap::algorithms::transforms::Transform(0, 0, 0, 0, -1.5, 0),
              .time = 0.0f,
          },
          {
              .pose = tap::algorithms::transforms::Transform(0, 0, 0, 0, 1.5, 0),
              .time = 1.0f,
          }}}};

    for (float i = 0.1; i < 1 - 1e-5; i += 0.1)
    {
        Transform expected = tap::algorithms::transforms::Transform(0, 0, 0, 0, i * 3 - 1.5, 0);
        expectEq(t.atTime(i), expected);
    }
}

TEST(Trajectory6d, two_waypoint_roll_only)
{
    Trajectory6D<2> t{
        {{{
              .pose = tap::algorithms::transforms::Transform(),
              .time = 0.0f,
          },
          {
              .pose = tap::algorithms::transforms::Transform(0, 0, 0, 3, 0, 0),
              .time = 1.0f,
          }}}};

    for (float i = 0; i <= 1; i += 0.1)
    {
        Transform expected = tap::algorithms::transforms::Transform(0, 0, 0, i * 3, 0, 0);
        expectEq(t.atTime(i), expected);
    }
}

TEST(Trajectory6d, clamps_before_first_and_after_last_waypoint)
{
    Trajectory6D<2> trajectory{
        {{{.pose = Transform(1, 2, 3, 0.1f, 0.2f, 0.3f), .time = 1.0f},
          {.pose = Transform(4, 5, 6, 0.3f, 0.2f, 0.1f), .time = 3.0f}}}};

    expectEq(trajectory.atTime(-1.0f), trajectory.waypoints.front().pose);
    expectEq(trajectory.atTime(1.0f), trajectory.waypoints.front().pose);
    expectEq(trajectory.atTime(3.0f), trajectory.waypoints.back().pose);
    expectEq(trajectory.atTime(3.002f), trajectory.waypoints.back().pose);
    expectEq(trajectory.atTime(100.0f), trajectory.waypoints.back().pose);
}

TEST(Trajectory6d, single_waypoint_holds_pose_at_all_times)
{
    Trajectory6D<1> trajectory{{{{.pose = Transform(1, 2, 3, 0.1f, 0.2f, 0.3f), .time = 1.0f}}}};

    for (float time : {-1.0f, 0.0f, 1.0f, 2.0f, 100.0f})
    {
        expectEq(trajectory.atTime(time), trajectory.waypoints.front().pose);
    }
}

TEST(Trajectory6d, three_waypoints_interpolate_each_segment_and_shared_endpoint)
{
    Trajectory6D<3> trajectory{
        {{{.pose = Transform(0, 0, 0, 0, 0, 0), .time = 0.0f},
          {.pose = Transform(0.5f, 0.1f, 0.6f, 0, 0, 0), .time = 4.0f},
          {.pose = Transform(0.5f, 0.1f, 0.5f, 0, 0, 0), .time = 6.0f}}}};

    expectEq(trajectory.atTime(2.0f), Transform(0.25f, 0.05f, 0.3f, 0, 0, 0));
    expectEq(trajectory.atTime(4.0f), trajectory.waypoints[1].pose);
    expectEq(trajectory.atTime(5.0f), Transform(0.5f, 0.1f, 0.55f, 0, 0, 0));
    expectEq(trajectory.atTime(6.002f), trajectory.waypoints.back().pose);
}

namespace
{
class TestTrajectoryIKCommand : public TrajectoryIKCommand<2>
{
public:
    using TrajectoryIKCommand<2>::TrajectoryIKCommand;
    const Transform& getDesiredPose() const { return baseToFollowerDesired; }
};

class TrajectoryIKCommandTest : public Test
{
protected:
    void SetUp() override
    {
        for (auto* motor : {&yawMotor, &pitchMotor, &extensionMotor, &wrist1, &wrist2, &wrist3})
        {
            ON_CALL(*motor, isMotorOnline()).WillByDefault(Return(true));
            ON_CALL(*motor->getEncoder(), getPosition())
                .WillByDefault(Return(tap::algorithms::Angle(0)));
        }
        ON_CALL(wristEncoder, isOnline()).WillByDefault(Return(true));
        ON_CALL(wristEncoder, getPosition()).WillByDefault(Return(tap::algorithms::Angle(0)));
        clock.time = 1000;
    }

    tap::arch::clock::ClockStub clock;
    tap::Drivers drivers;
    NiceMock<tap::mock::MotorInterfaceMock> yawMotor, pitchMotor, extensionMotor;
    NiceMock<tap::mock::MotorInterfaceMock> wrist1, wrist2, wrist3;
    NiceMock<tap::mock::EncoderInterfaceMock> wristEncoder;
    const aruwsrc::control::turret::TurretMotorConfig motorConfig{
        .startAngle = 0,
        .startEncoderValue = 0,
        .minAngle = -M_PI,
        .maxAngle = M_PI,
        .limitMotorAngles = false};
    aruwsrc::control::turret::TurretMotor yaw{&yawMotor, motorConfig};
    aruwsrc::control::turret::TurretMotor pitch{&pitchMotor, motorConfig};
    aruwsrc::control::turret::TurretSubsystem turret{&drivers, pitch, yaw, nullptr};
    aruwsrc::control::joint::JointSubsystem extension{
        &drivers,
        extensionMotor,
        {.lowerBound = 0, .upperBound = 1, .posPidConfig = {}, .maxOutput = 0}};
    aruwsrc::engineer::wrist::WristSubsystem wrist{
        &drivers,
        wrist1,
        wrist2,
        wrist3,
        wristEncoder,
        {.theta1PidConfig = {},
         .theta2PidConfig = {},
         .theta3PidConfig = {},
         .theta2Min = -M_PI,
         .theta2Max = M_PI,
         .maxMotorDesiredOutput = 0}};
    aruwsrc::control::turret::algorithms::ChassisFrameTurretController<Axis::YAW> yawController{
        yaw,
        {}};
    aruwsrc::control::turret::algorithms::ChassisFrameTurretController<Axis::PITCH> pitchController{
        pitch,
        {}};
    Transform identity;
    Trajectory6D<2> trajectory{
        {{{.pose = Transform(0.3f, 0, 0.5f, 0, 0, 0), .time = 0.0f},
          {.pose = Transform(0.5f, 0.1f, 0.5f, 0, 0, 0), .time = 2.0f}}}};
    TestTrajectoryIKCommand command{
        identity,
        identity,
        turret,
        extension,
        wrist,
        yawController,
        pitchController,
        trajectory};
    TestTrajectoryIKCommand heldCommand{
        identity,
        identity,
        turret,
        extension,
        wrist,
        yawController,
        pitchController,
        trajectory,
        true};
};

TEST_F(TrajectoryIKCommandTest, initializes_controllers_and_starts_at_first_pose)
{
    command.initialize();
    EXPECT_EQ(&yawController, yaw.getTurretController());
    EXPECT_EQ(&pitchController, pitch.getTurretController());
    expectEq(command.getDesiredPose(), trajectory.waypoints.front().pose);
}

TEST_F(TrajectoryIKCommandTest, encoder_start_pose_does_not_step_yaw_when_imu_reference_differs)
{
    NiceMock<tap::mock::Odometry2DInterfaceMock> odometry;
    NiceMock<tap::mock::AbstractImuMock> chassisImu, turretImu;
    aruwsrc::engineer::algorithms::EngineerTransforms
        transforms(odometry, chassisImu, turret, turretImu, extension, wrist, extension);

    // The world/IMU heading need not match the encoder zero. Chassis-relative FK must not read it.
    ON_CALL(turretImu, getYaw()).WillByDefault(Return(2.0f));
    EXPECT_CALL(turretImu, getYaw()).Times(0);
    EXPECT_CALL(turretImu, getPitch()).Times(0);

    for (float measuredYaw : {0.0f, 0.7f, -0.7f, 3.13f, -3.13f})
    {
        ON_CALL(*yawMotor.getEncoder(), getPosition())
            .WillByDefault(Return(tap::algorithms::Angle(measuredYaw)));
        ON_CALL(*pitchMotor.getEncoder(), getPosition())
            .WillByDefault(Return(tap::algorithms::Angle(-0.2f)));
        ON_CALL(*extensionMotor.getEncoder(), getPosition())
            .WillByDefault(Return(tap::algorithms::Angle(0.2f)));
        yaw.updateMotorAngle();
        pitch.updateMotorAngle();

        trajectory.waypoints.front().pose = transforms.getChassisToEndEffector();
        command.initialize();
        command.execute();

        EXPECT_NEAR(
            yawController.getSetpoint().minDifference(tap::algorithms::Angle(measuredYaw)),
            0,
            EPS);
        EXPECT_NEAR(
            pitchController.getSetpoint().minDifference(tap::algorithms::Angle(-0.2f)),
            0,
            EPS);
    }
}

TEST_F(TrajectoryIKCommandTest, finishes_at_endpoint_and_samples_safely_after_overrun)
{
    command.initialize();
    clock.time = 2999;
    EXPECT_FALSE(command.isFinished());
    clock.time = 3000;
    EXPECT_TRUE(command.isFinished());
    clock.time = 3002;
    command.updateBaseToFollowerDesired();
    expectEq(command.getDesiredPose(), trajectory.waypoints.back().pose);
}

TEST_F(TrajectoryIKCommandTest, held_command_does_not_finish_or_restart_and_can_be_reinitialized)
{
    heldCommand.initialize();
    clock.time = 10000;
    EXPECT_FALSE(heldCommand.isFinished());
    heldCommand.updateBaseToFollowerDesired();
    expectEq(heldCommand.getDesiredPose(), trajectory.waypoints.back().pose);

    heldCommand.initialize();
    expectEq(heldCommand.getDesiredPose(), trajectory.waypoints.front().pose);
    clock.time = 11000;
    heldCommand.updateBaseToFollowerDesired();
    expectEq(heldCommand.getDesiredPose(), trajectory.atTime(1.0f));
}

TEST_F(TrajectoryIKCommandTest, requires_all_motors_online)
{
    EXPECT_TRUE(command.isReady());
    ON_CALL(extensionMotor, isMotorOnline()).WillByDefault(Return(false));
    EXPECT_FALSE(command.isReady());
}

TEST_F(TrajectoryIKCommandTest, switch_release_stops_outputs_and_holds_measured_joint_positions)
{
    extension.setSetpoint(0.7f);
    wrist.setSetpointTheta1(0.2f);
    wrist.setSetpointTheta2(0.3f);
    wrist.setSetpointTheta3(0.4f);

    EXPECT_CALL(yawMotor, setDesiredOutput(0));
    EXPECT_CALL(pitchMotor, setDesiredOutput(0));
    // whileTrue trigger cancellation uses end(false), not end(true).
    heldCommand.end(false);
    extension.updateSetpoint();
    EXPECT_FLOAT_EQ(extension.getPosition(), extension.getSetpoint());
    EXPECT_FLOAT_EQ(wrist.getTheta1(), wrist.getSetpointTheta1());
    EXPECT_FLOAT_EQ(wrist.getTheta2(), wrist.getSetpointTheta2());
    EXPECT_FLOAT_EQ(wrist.getTheta3(), wrist.getSetpointTheta3());
}
}  // namespace
