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

#include "turret_cv_command.hpp"

#include "tap/algorithms/ballistics.hpp"
#include "tap/architecture/clock.hpp"

#include "../algorithms/chassis_frame_turret_controller.hpp"
#include "../robot_turret_subsystem.hpp"
#include "aruwsrc/algorithms/odometry/otto_velocity_odometry_2d_subsystem.hpp"
#include "aruwsrc/control/launcher/referee_feedback_friction_wheel_subsystem.hpp"

using namespace tap::arch::clock;
using namespace tap::algorithms;

using aruwsrc::algorithms::ballistics::CvBallisticsSolver;

namespace aruwsrc::control::turret::cv
{
TurretCVCommand::TurretCVCommand(
    communication::serial::VisionCoprocessor *visionCoprocessor,
    control::ControlOperatorInterface *controlOperatorInterface,
    RobotTurretSubsystem *turretSubsystem,
    algorithms::TurretAxisControllerInterface<tap::algorithms::transforms::Axis::YAW>
        *yawController,
    algorithms::TurretAxisControllerInterface<tap::algorithms::transforms::Axis::PITCH>
        *pitchController,
    CvBallisticsSolver *ballisticsSolver,
    const float userYawInputScalar,
    const float userPitchInputScalar,
    uint8_t turretID)
    : TurretCVCommand(
          visionCoprocessor,
          controlOperatorInterface,
          turretSubsystem,
          yawController,
          pitchController,
          ballisticsSolver,
          userYawInputScalar,
          userPitchInputScalar,
          turretID,
          ScanConfig{})
{
}

TurretCVCommand::TurretCVCommand(
    communication::serial::VisionCoprocessor *visionCoprocessor,
    control::ControlOperatorInterface *controlOperatorInterface,
    RobotTurretSubsystem *turretSubsystem,
    algorithms::TurretAxisControllerInterface<tap::algorithms::transforms::Axis::YAW>
        *yawController,
    algorithms::TurretAxisControllerInterface<tap::algorithms::transforms::Axis::PITCH>
        *pitchController,
    CvBallisticsSolver *ballisticsSolver,
    const float userYawInputScalar,
    const float userPitchInputScalar,
    uint8_t turretID,
    const ScanConfig &scanConfig)
    : visionCoprocessor(visionCoprocessor),
      controlOperatorInterface(controlOperatorInterface),
      turretID(turretID),
      turretSubsystem(turretSubsystem),
      yawController(yawController),
      pitchController(pitchController),
      ballisticsSolver(ballisticsSolver),
      userYawInputScalar(userYawInputScalar),
      userPitchInputScalar(userPitchInputScalar),
      scanConfig(scanConfig)
{
    assert(ballisticsSolver != nullptr);

    assert(turretID == ballisticsSolver->getTurretID());
    addSubsystemRequirement(turretSubsystem);
}

bool TurretCVCommand::isReady() { return !isFinished(); }

void TurretCVCommand::initialize()
{
    pitchController->initialize();
    yawController->initialize();
    prevTime = getTimeMilliseconds();
    visionCoprocessor->sendSelectNewTargetMessage();
}

void TurretCVCommand::enterScanMode(const WrappedFloat &yawSetpoint)
{
    scanning = true;
    yawScanCenter = yawSetpoint;
    yawScanOffset = 0.0f;
    pitchScanValue = scanConfig.pitchScanMinAngle;
    yawScanDirection = 1;
    pitchScanDirection = 1;
}

void TurretCVCommand::exitScanMode()
{
    scanning = false;
    lostTargetCounter = 0;
}

void TurretCVCommand::updateScanSetpoints(WrappedFloat *yawSetpoint, WrappedFloat *pitchSetpoint)
{
    yawScanOffset += scanConfig.yawScanDeltaAngle * yawScanDirection;
    yawScanOffset = tap::algorithms::limitVal(
        yawScanOffset,
        -scanConfig.yawScanHalfRange,
        scanConfig.yawScanHalfRange);

    pitchScanValue += scanConfig.pitchScanDeltaAngle * pitchScanDirection;
    if (pitchScanValue >= scanConfig.pitchScanMaxAngle)
    {
        pitchScanValue = scanConfig.pitchScanMaxAngle;
        pitchScanDirection = -1;
    }
    else if (pitchScanValue <= scanConfig.pitchScanMinAngle)
    {
        pitchScanValue = scanConfig.pitchScanMinAngle;
        pitchScanDirection = 1;
    }

    const WrappedFloat targetYawSetpoint = yawScanCenter + yawScanOffset;
    *yawSetpoint = yawSetpoint->minInterpolate(targetYawSetpoint, scanConfig.lowPassAlpha);
    *pitchSetpoint = pitchSetpoint->minInterpolate(Angle(pitchScanValue), scanConfig.lowPassAlpha);

    const bool yawAtEndpoint =
        abs(yawSetpoint->minDifference(targetYawSetpoint)) < scanConfig.endpointTolerance;
    if (yawAtEndpoint && yawScanOffset >= scanConfig.yawScanHalfRange)
    {
        yawScanDirection = -1;
    }
    else if (yawAtEndpoint && yawScanOffset <= -scanConfig.yawScanHalfRange)
    {
        yawScanDirection = 1;
    }
}

void TurretCVCommand::execute()
{
    WrappedFloat pitchSetpoint = pitchController->getSetpoint();
    WrappedFloat yawSetpoint = yawController->getSetpoint();

    std::optional<CvBallisticsSolver::BallisticsSolution> ballisticsSolution =
        ballisticsSolver->computeTurretAimAngles();

    if (ballisticsSolution != std::nullopt)
    {
        exitScanMode();
        pitchSetpoint = Angle(ballisticsSolution->pitchAngle);
        yawSetpoint = Angle(ballisticsSolution->yawAngle);

        /**
         * the setpoint returned by the ballistics solver is between [0, 2*PI), so find the
         * setpoint that is closest to the wrapped measured angle.
         */
        withinAimingTolerance = CvBallisticsSolver::withinAimingTolerance(
            yawController->getMeasurement().minDifference(yawSetpoint),
            pitchController->getMeasurement().minDifference(pitchSetpoint),
            ballisticsSolution->distance);
    }
    else
    {
        withinAimingTolerance = false;

        if (scanConfig.enabled)
        {
            if (lostTargetCounter < scanConfig.aimLostNumCounts)
            {
                lostTargetCounter++;
            }
            else
            {
                if (!scanning)
                {
                    enterScanMode(yawSetpoint);
                }
                updateScanSetpoints(&yawSetpoint, &pitchSetpoint);
            }
        }
        else
        {
            // No valid ballistics solution, let the user control the turret.
            pitchSetpoint +=
                userPitchInputScalar * controlOperatorInterface->getTurretPitchInput(turretID);

            yawSetpoint +=
                userYawInputScalar * controlOperatorInterface->getTurretYawInput(turretID);
        }
    }

    uint32_t currTime = getTimeMilliseconds();
    float dt = (currTime - prevTime) / 1000.0f;
    prevTime = currTime;

    // updates the turret pitch setpoint based on either CV or user input, runs the PID controller,
    // and sets the turret subsystem's desired pitch output
    pitchController->runController(dt, pitchSetpoint);

    // updates the turret yaw setpoint based on either CV or user input, runs the PID controller,
    // and sets the turret subsystem's desired yaw output
    yawController->runController(dt, yawSetpoint);
}

bool TurretCVCommand::isFinished() const
{
    return !pitchController->isOnline() || !yawController->isOnline();
}

void TurretCVCommand::end(bool)
{
    turretSubsystem->yawMotor.setMotorOutput(0);
    turretSubsystem->pitchMotor.setMotorOutput(0);
    withinAimingTolerance = false;
    exitScanMode();
}

}  // namespace aruwsrc::control::turret::cv
