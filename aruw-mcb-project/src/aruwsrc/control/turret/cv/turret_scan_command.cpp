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

#include "turret_scan_command.hpp"

#include "../algorithms/turret_controller_interface.hpp"
#include "../turret_subsystem.hpp"

using namespace tap::algorithms;

namespace aruwsrc::control::turret::cv
{
TurretScanCommand::TurretScanCommand(
    TurretSubsystem &turret,
    algorithms::TurretControllerInterface &yawController,
    algorithms::TurretControllerInterface &pitchController,
    const Config &config)
    : turret(turret),
      yawController(yawController),
      pitchController(pitchController),
      config(config),
      pitchScanner(config.pitchScanConfig),
      yawScanner(config.yawScanConfig)
{
    addSubsystemRequirement(&turret);
}

bool TurretScanCommand::isReady() { return yawController.isOnline() || pitchController.isOnline(); }

void TurretScanCommand::initialize()
{
    prevTime = tap::arch::clock::getTimeMilliseconds();

    yawController.initialize();
    pitchController.initialize();

    float yawSetpoint =
        yawController.convertControllerAngleToChassisFrame(yawController.getSetpoint());
    float pitchSetpoint =
        pitchController.convertControllerAngleToChassisFrame(pitchController.getSetpoint());

    yawScanner.setScanSetpoint(yawSetpoint);
    pitchScanner.setScanSetpoint(pitchSetpoint);

    lostTargetCounter = 0;
}

void TurretScanCommand::execute()
{
    uint32_t curTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t dt = curTime - prevTime;
    prevTime = curTime;

    if (lostTargetCounter < config.aimLostNumCounts)
    {
        lostTargetCounter++;

        yawController.runController(dt, yawController.getSetpoint());
        pitchController.runController(dt, pitchController.getSetpoint());
    }
    else
    {
        float yawScanValue = yawController.convertChassisAngleToControllerFrame(yawScanner.scan());
        float pitchScanValue =
            pitchController.convertChassisAngleToControllerFrame(pitchScanner.scan());

        yawScanValue = lowPassFilter(yawScanValue, yawScanValue, config.scanLowPassAlpha);
        pitchScanValue = lowPassFilter(pitchScanValue, pitchScanValue, config.scanLowPassAlpha);

        yawController.runController(dt, yawScanValue);
        pitchController.runController(dt, pitchScanValue);
    }
}

void TurretScanCommand::end(bool)
{
    turret.yawMotor.setMotorOutput(0);
    turret.pitchMotor.setMotorOutput(0);
}

bool TurretScanCommand::isFinished() const
{
    return !yawController.isOnline() && !pitchController.isOnline();
}
}  // namespace aruwsrc::control::turret::cv
