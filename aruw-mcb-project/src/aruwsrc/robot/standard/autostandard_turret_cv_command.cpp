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
#include "autostandard_turret_cv_command.hpp"

#include <cassert>

#include "tap/algorithms/ballistics.hpp"
#include "tap/algorithms/math_user_utils.hpp"
#include "tap/algorithms/wrapped_float.hpp"
#include "tap/architecture/clock.hpp"

#include "aruwsrc/algorithms/odometry/otto_velocity_odometry_2d_subsystem.hpp"
#include "aruwsrc/control/launcher/referee_feedback_friction_wheel_subsystem.hpp"
#include "aruwsrc/control/turret/cv/setpoint_scanner.hpp"
#include "aruwsrc/control/turret/robot_turret_subsystem.hpp"

using namespace tap::arch::clock;
using namespace tap::algorithms;
using namespace aruwsrc::algorithms;

namespace aruwsrc::standard
{
AutostandardTurretCVCommand::AutostandardTurretCVCommand(
    serial::VisionCoprocessor &visionCoprocessor,
    TurretConfig &turretConfig,
    aruwsrc::algorithms::transforms::StandardAndHeroTransformer &transformer)
    : visionCoprocessor(visionCoprocessor),
      turretConfig(turretConfig),
      transformer(transformer)
{
    this->addSubsystemRequirement(&turretConfig.turretSubsystem);
}

bool AutostandardTurretCVCommand::isReady() { return !isFinished(); }

void AutostandardTurretCVCommand::initialize()
{
    prevTime = getTimeMilliseconds();
    visionCoprocessor.sendSelectNewTargetMessage();
}

void AutostandardTurretCVCommand::computeAimSetpoints(
    TurretConfig &config,
    aruwsrc::algorithms::OttoBallisticsSolver::BallisticsSolution &solution,
    WrappedFloat *desiredYawSetpoint,
    WrappedFloat *desiredPitchSetpoint,
    bool *withinAimingTolerance)
{
    // Get world-relative setpoints
    *desiredYawSetpoint = Angle(solution.yawAngle);
    *desiredPitchSetpoint = Angle(solution.pitchAngle);

    *withinAimingTolerance = turretConfig.ballisticsSolver.withinAimingTolerance(
        config.yawController.getMeasurement().minDifference(*desiredYawSetpoint),
        config.pitchController.getMeasurement().minDifference(*desiredPitchSetpoint),
        solution.distance);
}

void AutostandardTurretCVCommand::execute()
{
    // setpoints are in chassis frame
    WrappedFloat yawSetpoint = turretConfig.yawController.getSetpoint();
    WrappedFloat pitchSetpoint = turretConfig.pitchController.getSetpoint();

    auto ballisticsSolution = turretConfig.ballisticsSolver.computeTurretAimAngles();

    // @todo: does not allow for independent turret aiming
    targetFound = (ballisticsSolution != std::nullopt);

    // Turret minor control
    // If target spotted
    if (targetFound)
    {
        exitScanMode();

        if (ballisticsSolution != std::nullopt)
        {
            computeAimSetpoints(
                turretConfig,
                ballisticsSolution.value(),
                &yawSetpoint,
                &pitchSetpoint,
                &withinAimingTolerance);
        }
    }
    else
    {
        withinAimingTolerance = false;

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
                enterScanMode();
            }

            // scan logic: start at some default, scan 180deg clockwise, change direction
            // scan 180 ccw, change, etc.
            float v = scanValue.getWrappedValue();
            if (v >= CCW_TO_CW_WRAP_VALUE)
                scanDir = SCAN_CLOCKWISE;  // decreases angle
            else if (v <= CW_TO_CCW_WRAP_VALUE)
                scanDir = SCAN_COUNTER_CLOCKWISE;  // increases angle

            pitchSetpoint = Angle(SCAN_TURRET_PITCH);
            yawSetpoint = scanValue;
        }
    }

    uint32_t currTime = getTimeMilliseconds();
    uint32_t dt = currTime - prevTime;
    prevTime = currTime;

    turretConfig.pitchController.runController(dt, pitchSetpoint);
    turretConfig.yawController.runController(dt, yawSetpoint);
}

bool AutostandardTurretCVCommand::isFinished() const
{
    return !turretConfig.pitchController.isOnline() || !turretConfig.yawController.isOnline();
}

void AutostandardTurretCVCommand::end(bool)
{
    turretConfig.turretSubsystem.pitchMotor.setMotorOutput(0);
    turretConfig.turretSubsystem.yawMotor.setMotorOutput(0);

    withinAimingTolerance = false;
    exitScanMode();
}

void AutostandardTurretCVCommand::requestNewTarget()
{
    visionCoprocessor.sendSelectNewTargetMessage();
}

}  // namespace aruwsrc::standard
