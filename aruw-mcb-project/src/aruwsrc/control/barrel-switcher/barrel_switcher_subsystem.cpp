/*
 * Copyright (c) 2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

// Inspired by TAMU barrel switcher

#include "barrel_switcher_subsystem.hpp"

#include <cassert>

namespace aruwsrc::control::barrel_switcher
{
BarrelSwitcherSubsystem::BarrelSwitcherSubsystem(
    tap::Drivers& drivers,
    const BarrelSwitcherMotorConfig& motorConfig,
    const BarrelSwitcherConfig& config,
    const tap::algorithms::SmoothPidConfig& pidConfig)
    : tap::control::Subsystem(&drivers),
      swapMotor(
          &drivers,
          motorConfig.motorId,
          motorConfig.canBus,
          motorConfig.isInverted,
          "Barrel Swap Motor"),
      positionPid(pidConfig),
      config(config),
      currentBarrel(tap::communication::serial::RefSerial::Rx::TURRET_17MM_1)
{
}

void BarrelSwitcherSubsystem::initialize()
{
    swapMotor.initialize();
    swapMotor.setDesiredOutput(0);
    currentSpikeTimer.execute();
}

void BarrelSwitcherSubsystem::refresh()
{
    if (swapMotor.isMotorOnline())
    {
        if (calibrationRequested)
        {
            performCalibration();
        }
        else
        {
            // run position pid controller
            float positionControllerError =
                getDesiredBarrelPosition() - getCalibratedMotorPosition();

            auto curTime = tap::arch::clock::getTimeMilliseconds();
            float dt = curTime - prevTime;
            prevTime = curTime;

            float out =
                positionPid.runController(positionControllerError, swapMotor.getShaftRPM(), dt);

            swapMotor.setDesiredOutput(out);
        }

        if (isBarrelAligned() && currentBarrelSide != BarrelSide::CALIBRATING)
        {
            currentBarrel = config.barrelArray[static_cast<int>(currentBarrelSide)];
        }
    }
}

void BarrelSwitcherSubsystem::performCalibration()
{
    // Slam into each wall and find current spike. Save position
    swapMotor.setDesiredOutput(-config.leadScrewCaliOutput);

    if (currentSpikeTimer.execute() &&
        abs(swapMotor.getTorque()) >= config.leadScrewCurrentSpikeTorque)
    {
        // finished, found left side.
        swapMotor.setDesiredOutput(0);

        leftSideCalibrationPosition = getRawPosition();

        currentBarrelSide = BarrelSide::LEFT;

        calibrationRequested = false;
    }

    if (abs(swapMotor.getTorque()) >= config.leadScrewCurrentSpikeTorque &&
        (currentSpikeTimer.isExpired() || currentSpikeTimer.isStopped()))
    {
        currentSpikeTimer.restart(500);
    }
}

BarrelSide BarrelSwitcherSubsystem::getSide() const { return currentBarrelSide; }

void BarrelSwitcherSubsystem::toggleSide()
{
    currentBarrelSide =
        (currentBarrelSide == BarrelSide::LEFT) ? BarrelSide::RIGHT : BarrelSide::LEFT;
}

bool BarrelSwitcherSubsystem::isBarrelAligned() const
{
    return abs(getCalibratedMotorPosition() - getDesiredBarrelPosition()) <=
           config.barrelsAlignedToleranceMM;
}

float BarrelSwitcherSubsystem::getDesiredBarrelPosition() const
{
    switch (getSide())
    {
        case BarrelSide::LEFT:
            return getLeftSidePosition();
        case BarrelSide::RIGHT:
            return getRightSidePosition();
        case BarrelSide::CALIBRATING:
            return 0;
        default:
            assert(false);
    }
}

float BarrelSwitcherSubsystem::getRawPosition() const
{
    return swapMotor.getEncoderUnwrapped() / config.leadScrewTicksPerMM;
}

float BarrelSwitcherSubsystem::getCalibratedMotorPosition() const
{
    return getRawPosition() - leftSideCalibrationPosition;
}

}  // namespace aruwsrc::control::barrel_switcher
