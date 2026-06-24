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

#include <aruwsrc/algorithms/plate_hit_tracker.hpp>

#include "tap/algorithms/ballistics.hpp"
#include "tap/algorithms/math_user_utils.hpp"
#include "tap/algorithms/wrapped_float.hpp"
#include "tap/architecture/clock.hpp"

#include "aruwsrc/algorithms/odometry/otto_velocity_odometry_2d_subsystem.hpp"
#include "aruwsrc/control/launcher/referee_feedback_friction_wheel_subsystem.hpp"
#include "aruwsrc/control/turret/cv/setpoint_scanner.hpp"
#include "aruwsrc/control/turret/robot_turret_subsystem.hpp"
#include "aruwsrc/robot/sentry/turret/sentry_turret_minor_subsystem.hpp"

using namespace tap::arch::clock;
using namespace tap::algorithms;
using namespace aruwsrc::algorithms;

namespace aruwsrc::sentry::turret::cv
{
SentryTurretCVCommand::SentryTurretCVCommand(
    communication::serial::VisionCoprocessor &visionCoprocessor,
    aruwsrc::algorithms::PlateHitTracker &plateHitTracker,
    aruwsrc::control::turret::YawTurretSubsystem &turretMajorSubsystem,
    aruwsrc::control::turret::algorithms::TurretAxisControllerInterface<
        aruwsrc::control::turret::algorithms::Axis::YAW> &yawControllerMajor,
    TurretConfig &turretWidowConfig,
    aruwsrc::sentry::algorithms::odometry::SentryTransforms &sentryTransforms)
    : visionCoprocessor(visionCoprocessor),
      plateHitTracker(plateHitTracker),
      turretMajorSubsystem(turretMajorSubsystem),
      yawControllerMajor(yawControllerMajor),
      turretWidowConfig(turretWidowConfig),
      sentryTransforms(sentryTransforms)
{
    this->addSubsystemRequirement(&turretMajorSubsystem);
    this->addSubsystemRequirement(&turretWidowConfig.turretSubsystem);
}

bool SentryTurretCVCommand::isReady() { return !isFinished(); }

void SentryTurretCVCommand::initialize()
{
    prevTime = getTimeMilliseconds();
    visionCoprocessor.sendSelectNewTargetMessage();
}

void SentryTurretCVCommand::computeAimSetpoints(
    TurretConfig &config,
    aruwsrc::sentry::algorithms::SentryBallisticsSolver::BallisticsSolution &solution,
    WrappedFloat *desiredYawSetpoint,
    WrappedFloat *desiredPitchSetpoint,
    bool *withinAimingTolerance)
{
    // Get world-relative setpoints
    *desiredYawSetpoint = Angle(solution.yawAngle);
    *desiredPitchSetpoint = Angle(solution.pitchAngle);

    *withinAimingTolerance = config.ballisticsSolver.withinAimingTolerance(
        config.yawController.getMeasurement().minDifference(*desiredYawSetpoint),
        config.pitchController.getMeasurement().minDifference(*desiredPitchSetpoint),
        solution.distance);
}

void SentryTurretCVCommand::execute()
{
    // setpoints are in chassis frame
    WrappedFloat majorSetpoint = yawControllerMajor.getSetpoint();
    WrappedFloat widowYawSetpoint = turretWidowConfig.yawController.getSetpoint();
    WrappedFloat widowPitchSetpoint = turretWidowConfig.pitchController.getSetpoint();

    auto widowBallisticsSolution = turretWidowConfig.ballisticsSolver.computeTurretAimAngles();

    targetFound = (widowBallisticsSolution != std::nullopt);

    // Turret minor control
    // If target spotted
    if (targetFound)
    {
        exitScanMode();

        if (widowBallisticsSolution != std::nullopt)
        {
            computeAimSetpoints(
                turretWidowConfig,
                widowBallisticsSolution.value(),
                &widowYawSetpoint,
                &widowPitchSetpoint,
                &withinAimingToleranceWidow);
        }

        majorSetpoint = widowYawSetpoint;
    }
    else
    {
        withinAimingToleranceWidow = false;

        // See how recently we lost target
        if (lostTargetCounter < AIM_LOST_NUM_COUNTS)
        {
            // We recently had a target. Don't start scanning yet
            lostTargetCounter++;
            // Pitch and yaw setpoint already at reasonable default value
            // by this point
        }
        else
        {
            // Scan
            if (!scanning)
            {
                enterScanMode(majorSetpoint, widowYawSetpoint);
            }

            if (curHitState == HitState::NOT_HIT)
            {
                scanOffsetFromCenter += YAW_SCAN_DELTA_ANGLE * scanDir;
                if (scanOffsetFromCenter >= YAW_SCAN_HALF_RANGE)
                {
                    scanOffsetFromCenter = YAW_SCAN_HALF_RANGE;
                }
                else if (scanOffsetFromCenter <= -YAW_SCAN_HALF_RANGE)
                {
                    scanOffsetFromCenter = -YAW_SCAN_HALF_RANGE;
                }

                pitchScanValue += PITCH_SCAN_DELTA_ANGLE * pitchScanDir;
                if (pitchScanValue >= SCAN_TURRET_MINOR_DOWN_PITCH)
                {
                    pitchScanValue = SCAN_TURRET_MINOR_DOWN_PITCH;
                    pitchScanDir = SCAN_CLOCKWISE;
                }
                else if (pitchScanValue <= SCAN_TURRET_MINOR_UP_PITCH)
                {
                    pitchScanValue = SCAN_TURRET_MINOR_UP_PITCH;
                    pitchScanDir = SCAN_COUNTER_CLOCKWISE;
                }

                minorScanValue = scanCenter + scanOffsetFromCenter;
                widowPitchSetpoint =
                    widowPitchSetpoint.minInterpolate(Angle(pitchScanValue), SCAN_LOW_PASS_ALPHA);
                widowYawSetpoint =
                    widowYawSetpoint.minInterpolate(minorScanValue, SCAN_LOW_PASS_ALPHA);

                majorScanValue = scanCenter + scanOffsetFromCenter * MAJOR_SCAN_RATIO;
                majorSetpoint =
                    majorSetpoint.minInterpolate(majorScanValue, SCAN_LOW_PASS_ALPHA_MAJOR);

                const bool scanSetpointsAtEndpoint =
                    abs(widowYawSetpoint.minDifference(minorScanValue)) < SCAN_ENDPOINT_TOLERANCE &&
                    abs(majorSetpoint.minDifference(majorScanValue)) < SCAN_ENDPOINT_TOLERANCE;

                if (scanOffsetFromCenter >= YAW_SCAN_HALF_RANGE && scanSetpointsAtEndpoint)
                {
                    scanDir = SCAN_CLOCKWISE;
                }
                else if (scanOffsetFromCenter <= -YAW_SCAN_HALF_RANGE && scanSetpointsAtEndpoint)
                {
                    scanDir = SCAN_COUNTER_CLOCKWISE;
                }
            }
        }
    }

    const std::vector<PlateHitTracker::PlateHitBinData> &hitData =
        plateHitTracker.getPeakAnglesRadians();
    PlateHitTracker::PlateHitBinData maxHit;
    if (!hitData.empty())
    {
        maxHit = hitData[0];
    }
    else
    {
        maxHit.magnitude = 0.0f;
        maxHit.radians = Angle(0);
        maxHit.projectileType = PlateHitTracker::ProjectileType::NONE;
    }

    lastPlateHitData = plateHitData;
    plateHitData = maxHit;
    switch (curHitState)
    {
        case HitState::HIT:
        {
            // set new setpoint if hit state transition or new hit is registered
            hitLocDiffRads =
                abs(plateHitData.radians.getUnwrappedValue() -
                    lastPlateHitData.radians.getUnwrappedValue());
            if (scanning && (lastHitState != curHitState || hitLocDiffRads > HIT_DIFF_OFFSET))
            {
                majorSetpoint = maxHit.radians;
                widowYawSetpoint = maxHit.radians;
            }
            lastHitState = curHitState;
            uint32_t curTime = tap::arch::clock::getTimeMilliseconds();
            if (maxHit.magnitude < HIT_MAG_THRESH &&
                curTime - lastHitTime > HIT_COUNT_DELAY_MILLISEC)
            {
                curHitState = HitState::NOT_HIT;
            }
            else if (maxHit.magnitude >= HIT_MAG_THRESH)
            {
                lastHitTime = curTime;
            }
            break;
        }
        case HitState::NOT_HIT:
        {
            lastHitState = curHitState;
            if (maxHit.magnitude >= HIT_MAG_THRESH)
            {
                curHitState = HitState::HIT;
                if (scanning)
                {
                    majorSetpoint = maxHit.radians;
                    widowYawSetpoint = maxHit.radians;
                }
            }
            break;
        }
        default:
            break;
    }

    uint32_t currTime = getTimeMilliseconds();
    float dt = (currTime - prevTime) / 1000.0f;
    prevTime = currTime;

    yawControllerMajor.runController(dt, majorSetpoint);
    turretWidowConfig.pitchController.runController(dt, widowPitchSetpoint);
    turretWidowConfig.yawController.runController(dt, widowYawSetpoint);
}

bool SentryTurretCVCommand::isFinished() const
{
    return !turretWidowConfig.pitchController.isOnline() ||
           !turretWidowConfig.yawController.isOnline();
}

void SentryTurretCVCommand::end(bool)
{
    turretMajorSubsystem.getMutableMotor().setMotorOutput(0);

    turretWidowConfig.turretSubsystem.pitchMotor.setMotorOutput(0);
    turretWidowConfig.turretSubsystem.yawMotor.setMotorOutput(0);
    withinAimingToleranceWidow = false;
    exitScanMode();
}

void SentryTurretCVCommand::requestNewTarget() { visionCoprocessor.sendSelectNewTargetMessage(); }

}  // namespace aruwsrc::sentry::turret::cv
