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

#include "aruwlib/algorithms/math_user_utils.hpp"
#include "aruwlib/architecture/clock.hpp"
#include "aruwlib/communication/remote.hpp"
#include "aruwlib/drivers.hpp"

using namespace aruwlib::arch::clock;

namespace aruwsrc::control::turret
{
TurretCVCommand::TurretCVCommand(
    aruwlib::Drivers *drivers,
    TurretSubsystem *subsystem,
    float turretStartAngle,
    float yawKp,
    float yawKi,
    float yawKd,
    float yawMaxICumulative,
    float yawMaxOutput,
    float yawTQDerivativeKalman,
    float yawTRDerivativeKalman,
    float yawTQProportionalKalman,
    float yawTRProportionalKalman,
    float pitchKp,
    float pitchKi,
    float pitchKd,
    float pitchMaxICumulative,
    float pitchMaxOutput,
    float pitchTQDerivativeKalman,
    float pitchTRDerivativeKalman,
    float pitchTQProportionalKalman,
    float pitchTRProportionalKalman)
    : drivers(drivers),
      turretSubsystem(subsystem),
      yawTargetAngle(turretStartAngle, 0.0f, 360.0f),
      pitchTargetAngle(turretStartAngle, 0.0f, 360.0f),
      yawPid(
          yawKp,
          yawKi,
          yawKd,
          yawMaxICumulative,
          yawMaxOutput,
          yawTQDerivativeKalman,
          yawTRDerivativeKalman,
          yawTQProportionalKalman,
          yawTRProportionalKalman),
      pitchPid(
          pitchKp,
          pitchKi,
          pitchKd,
          pitchMaxICumulative,
          pitchMaxOutput,
          pitchTQDerivativeKalman,
          pitchTRDerivativeKalman,
          pitchTQProportionalKalman,
          pitchTRProportionalKalman)
{
    addSubsystemRequirement(dynamic_cast<aruwlib::control::Subsystem *>(subsystem));
}

void TurretCVCommand::initialize()
{
    drivers->xavierSerial.beginAutoAim();
    yawPid.reset();
    pitchPid.reset();
}

void TurretCVCommand::execute()
{
    if (drivers->xavierSerial.lastAimDataValid())
    {
        const auto &cvData = drivers->xavierSerial.getLastAimData();
        if (cvData.hasTarget)
        {
            turretSubsystem->setYawSetpoint(cvData.yaw);
            turretSubsystem->setPitchSetpoint(cvData.pitch);
        }
    }

    uint32_t currTime = getTimeMilliseconds();
    uint32_t dt = currTime - prevTime;
    prevTime = currTime;
    runYawPositionController(dt);
    runPitchPositionController(dt);
}

void TurretCVCommand::end(bool) { drivers->xavierSerial.stopAutoAim(); }

void TurretCVCommand::runYawPositionController(float dt)
{
    // position controller based on gimbal angle
    float positionControllerError =
        turretSubsystem->getCurrentYawValue().difference(turretSubsystem->getYawSetpoint());
    float pidOutput =
        yawPid.runController(positionControllerError, turretSubsystem->getYawVelocity(), dt);

    turretSubsystem->setYawMotorOutput(pidOutput);
}

void TurretCVCommand::runPitchPositionController(float dt)
{
    // position controller based on turret pitch gimbal
    float positionControllerError =
        turretSubsystem->getCurrentPitchValue().difference(turretSubsystem->getPitchSetpoint());
    float pidOutput =
        pitchPid.runController(positionControllerError, turretSubsystem->getPitchVelocity(), dt);

    turretSubsystem->setPitchMotorOutput(pidOutput);
}

}  // namespace aruwsrc::control::turret
