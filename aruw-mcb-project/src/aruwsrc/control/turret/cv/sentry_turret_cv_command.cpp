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
#include "sentry_turret_cv_command.hpp"

#include <cassert>

#include "tap/algorithms/ballistics.hpp"
#include "tap/algorithms/math_user_utils.hpp"
#include "tap/algorithms/wrapped_float.hpp"
#include "tap/architecture/clock.hpp"

#include "aruwsrc/algorithms/odometry/otto_velocity_odometry_2d_subsystem.hpp"
#include "aruwsrc/control/launcher/referee_feedback_friction_wheel_subsystem.hpp"
#include "aruwsrc/control/turret/cv/setpoint_scanner.hpp"
#include "aruwsrc/control/turret/robot_turret_subsystem.hpp"
#include "aruwsrc/robot/sentry/sentry_turret_minor_subsystem.hpp"

using namespace tap::arch::clock;
using namespace tap::algorithms;
using namespace aruwsrc::algorithms;

using GameType = tap::communication::serial::RefSerialData::Rx::GameType;
using GameStage = tap::communication::serial::RefSerialData::Rx::GameStage;
using GameData = tap::communication::serial::RefSerialData::Rx::GameData;

namespace aruwsrc::control::sentry
{
SentryTurretCVCommand::SentryTurretCVCommand(
    serial::VisionCoprocessor &visionCoprocessor,
    tap::communication::serial::RefSerial &refSerial,
    aruwsrc::control::turret::YawTurretSubsystem &turretMajorSubsystem,
    aruwsrc::control::turret::algorithms::TurretYawControllerInterface &yawControllerMajor,
    TurretConfig &turretLeftConfig,
    TurretConfig &turretRightConfig,
    aruwsrc::sentry::SentryTransforms &sentryTransforms)
    : visionCoprocessor(visionCoprocessor),
      refSerial(refSerial),
      turretMajorSubsystem(turretMajorSubsystem),
      yawControllerMajor(yawControllerMajor),
      turretLeftConfig(turretLeftConfig),
      turretRightConfig(turretRightConfig),
      sentryTransforms(sentryTransforms)
{
    this->addSubsystemRequirement(&turretMajorSubsystem);
    this->addSubsystemRequirement(&turretLeftConfig.turretSubsystem);
    this->addSubsystemRequirement(&turretRightConfig.turretSubsystem);
}

bool SentryTurretCVCommand::isReady() { return !isFinished(); }

void SentryTurretCVCommand::initialize()
{
    prevTime = getTimeMilliseconds();
    visionCoprocessor.sendSelectNewTargetMessage();
}

void SentryTurretCVCommand::computeAimSetpoints(
    TurretConfig &config,
    aruwsrc::sentry::SentryBallisticsSolver::BallisticsSolution &solution,
    WrappedFloat *desiredYawSetpoint,
    WrappedFloat *desiredPitchSetpoint,
    bool *withinAimingTolerance)
{
    // Get world-relative setpoints
    *desiredYawSetpoint = Angle(solution.yawAngle);
    *desiredPitchSetpoint = Angle(solution.pitchAngle);

    // convert world-relative setpoints to turret major frame

    /**
     * the setpoint returned by the ballistics solver is between [0, 2*PI)
     * the desired setpoint is unwrapped when motor angles are limited, so find the setpoint
     * that is closest to the unwrapped measured angle.
     */
    // *desiredYawSetpoint = config.turretSubsystem.yawMotor.unwrapTargetAngle(*desiredYawSetpoint);
    // *desiredPitchSetpoint =
    //     config.turretSubsystem.pitchMotor.unwrapTargetAngle(*desiredPitchSetpoint);

    *withinAimingTolerance = turretLeftConfig.ballisticsSolver.withinAimingTolerance(
        config.yawController.getMeasurement().minDifference(*desiredYawSetpoint),
        config.pitchController.getMeasurement().minDifference(*desiredPitchSetpoint),
        solution.distance);
}

WrappedFloat debug_majorSetpoint(Angle(0));
WrappedFloat debug_leftPitchSetpoint(Angle(0));
WrappedFloat debug_rightPitchSetpoint(Angle(0));
WrappedFloat debug_leftYawSetpoint(Angle(0));
WrappedFloat debug_rightYawSetpoint(Angle(0));
bool debug = false;

WrappedFloat majorSetpoint(Angle(0));
WrappedFloat leftYawSetpoint(Angle(0));
WrappedFloat rightYawSetpoint(Angle(0));
WrappedFloat leftPitchSetpoint(Angle(0));
WrappedFloat rightPitchSetpoint(Angle(0));
void SentryTurretCVCommand::execute()
{
    // setpoints are in chassis frame
    majorSetpoint = yawControllerMajor.getSetpoint();
    leftYawSetpoint = turretLeftConfig.yawController.getSetpoint();
    rightYawSetpoint = turretRightConfig.yawController.getSetpoint();
    leftPitchSetpoint = turretLeftConfig.pitchController.getSetpoint();
    rightPitchSetpoint = turretRightConfig.pitchController.getSetpoint();

    auto leftBallisticsSolution = turretLeftConfig.ballisticsSolver.computeTurretAimAngles();
    auto rightBallisticsSolution = turretRightConfig.ballisticsSolver.computeTurretAimAngles();

    // @todo: does not allow for independent turret aiming
    targetFound = visionCoprocessor.isCvOnline() && (leftBallisticsSolution != std::nullopt &&
                                                     rightBallisticsSolution != std::nullopt);

    if (targetFound)
    {
        // Turret minor control
        // If target spotted
        exitScanMode();

        if (leftBallisticsSolution != std::nullopt)
        {
            computeAimSetpoints(
                turretLeftConfig,
                leftBallisticsSolution.value(),
                &leftYawSetpoint,
                &leftPitchSetpoint,
                &withinAimingToleranceLeft);
        }

        if (rightBallisticsSolution != std::nullopt)
        {
            computeAimSetpoints(
                turretRightConfig,
                rightBallisticsSolution.value(),
                &rightYawSetpoint,
                &rightPitchSetpoint,
                &withinAimingToleranceRight);
        }

        // sus: one of these could be std::nullopt ?
        WrappedFloat leftYawWrapped(leftBallisticsSolution->yawAngle, 0, M_TWOPI);
        WrappedFloat rightYawWrapped(rightBallisticsSolution->yawAngle, 0, M_TWOPI);

        // major averaging
        majorSetpoint = leftYawWrapped.minInterpolate(rightYawWrapped, 0.5);
    }
    else
    {
        withinAimingToleranceLeft = false;
        withinAimingToleranceRight = false;
        const GameData gameData = refSerial.getGameData();

        // See how recently we lost target
        if (lostTargetCounter < AIM_LOST_NUM_COUNTS)
        {
            // We recently had a target. Don't start scanning yet
            lostTargetCounter++;
            // Pitch and yaw setpoint already at reasonable default value
            // by this point
        }
        else if (
            (gameData.gameType != GameType::UNKNOWN) && (gameData.gameStage != GameStage::IN_GAME))
        {
            // Don't scan before the match starts, just look cool or smth
            majorSetpoint = Angle(0);
            leftYawSetpoint = Angle(M_PI_2);
            rightYawSetpoint = Angle(-M_PI_2);
            leftPitchSetpoint = Angle(0);
            rightPitchSetpoint = Angle(0);
        }
        else
        {
            // Scan
            if (!scanning)
            {
                enterScanMode(majorSetpoint);
            }

            // scan logic: start at some default, scan 180deg clockwise, change direction
            // scan 180 ccw, change, etc.
            float v = majorScanValue.getWrappedValue();
            if (v >= CCW_TO_CW_WRAP_VALUE)
                scanDir = SCAN_CLOCKWISE;  // decreases angle
            else if (v <= CW_TO_CCW_WRAP_VALUE)
                scanDir = SCAN_COUNTER_CLOCKWISE;  // increases angle

            majorScanValue += YAW_SCAN_DELTA_ANGLE * scanDir;
            majorSetpoint = majorSetpoint.minInterpolate(
                majorScanValue,
                SCAN_LOW_PASS_ALPHA);  // lowpass filter
            leftPitchSetpoint = Angle(SCAN_TURRET_MINOR_PITCH);
            rightPitchSetpoint = Angle(SCAN_TURRET_MINOR_PITCH);

            // temp = WrappedFloat(sentryTransforms.getWorldToTurretMajor().getYaw(), 0.0f, M_TWOPI)
            //            .getWrappedValue();

            leftYawSetpoint = majorSetpoint + SCAN_TURRET_LEFT_YAW;
            // leftYawSetpoint =
            //     turretLeftConfig.turretSubsystem.yawMotor.unwrapTargetAngle(leftYawSetpoint);
            rightYawSetpoint = majorSetpoint + SCAN_TURRET_RIGHT_YAW;
            // rightYawSetpoint =
            //     turretRightConfig.turretSubsystem.yawMotor.unwrapTargetAngle(rightYawSetpoint);
        }
    }

    uint32_t currTime = getTimeMilliseconds();
    uint32_t dt = currTime - prevTime;
    prevTime = currTime;

    yawControllerMajor.runController(dt, majorSetpoint);

    turretLeftConfig.pitchController.runController(dt, leftPitchSetpoint);
    turretRightConfig.pitchController.runController(dt, rightPitchSetpoint);

    turretLeftConfig.yawController.runController(dt, leftYawSetpoint);
    turretRightConfig.yawController.runController(dt, rightYawSetpoint);
}

bool SentryTurretCVCommand::isFinished() const
{
    return !turretLeftConfig.pitchController.isOnline() ||
           !turretLeftConfig.yawController.isOnline() ||
           !turretRightConfig.pitchController.isOnline() ||
           !turretRightConfig.yawController.isOnline();
}

void SentryTurretCVCommand::end(bool)
{
    turretMajorSubsystem.getMutableMotor().setMotorOutput(0);

    turretLeftConfig.turretSubsystem.pitchMotor.setMotorOutput(0);
    turretRightConfig.turretSubsystem.pitchMotor.setMotorOutput(0);

    turretLeftConfig.turretSubsystem.yawMotor.setMotorOutput(0);
    turretRightConfig.turretSubsystem.yawMotor.setMotorOutput(0);

    withinAimingToleranceLeft = false;
    withinAimingToleranceRight = false;
    exitScanMode();
}

void SentryTurretCVCommand::requestNewTarget() { visionCoprocessor.sendSelectNewTargetMessage(); }

}  // namespace aruwsrc::control::sentry
