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

/*
 * Manoli (the author of this code) would like the reader to know:
 * He does not endorse writing code like this.
 * This is an abuse of taproot and aruw-mcb.
 * This is NOT real command-based programming.
 * 
 * And most importantly:
 * Fuck Robert Olomon.
 * Design and manufacture your darts better next time :(
*/

#include "entire_dart_code_base_command.hpp"

namespace aruwsrc::control::dart
{
    EntireDartCodeBaseCommand::EntireDartCodeBaseCommand(
        aruwsrc::Drivers *drivers,
        aruwsrc::control::turret::StepperTurretSubsystem& turret,
        aruwsrc::agitator::AgitatorSubsystem& bottomBarrelSled,
        aruwsrc::agitator::AgitatorSubsystem& topBarrelSled,
        ConfigAConfig configA,
        OffsetFromBottomBarrelFirstDart offsetFromBottomBarrelFirstDart
    ) : ComprisedCommand(drivers),
    drivers(drivers),
    turret(turret),
    bottomBarrelSled(bottomBarrelSled),
    topBarrelSled(topBarrelSled),
    TIME_DELAY_AFTER_LAUNCHING(configA.timeDelayAfterLaunching),
    launchBottomBarrelFirstDart(
        &bottomBarrelSled,
        configA.dartOneAgitatorSetpoint,
        configA.agitatorAngularSpeed,
        configA.agitatorSetpointTolerance,
        true,
        true
    ),
    launchBottomBarrelSecondDart(
        &bottomBarrelSled,
        configA.dartTwoAgitatorSetpoint,
        configA.agitatorAngularSpeed,
        configA.agitatorSetpointTolerance,
        true,
        true
    ),
    launchTopBarrelFirstDart(
        &topBarrelSled,
        configA.dartOneAgitatorSetpoint,
        configA.agitatorAngularSpeed,
        configA.agitatorSetpointTolerance,
        true,
        true
    ),
    launchTopBarrelSecondDart(
        &topBarrelSled,
        configA.dartTwoAgitatorSetpoint,
        configA.agitatorAngularSpeed,
        configA.agitatorSetpointTolerance,
        true,
        true
    ),
    bottomBarrelSecondDartPositionOffset(
        drivers,
        turret,
        offsetFromBottomBarrelFirstDart.bottomBarrelSecondDartPitch,
        offsetFromBottomBarrelFirstDart.bottomBarrelSecondDartYaw
    ),
    topBarrelFirstDartPositionOffset(
        drivers,
        turret,
        offsetFromBottomBarrelFirstDart.topBarrelFirstDartPitch,
        offsetFromBottomBarrelFirstDart.topBarrelFirstDartYaw
    ),
    topBarrelSecondDartPositionOffset(
        drivers,
        turret,
        offsetFromBottomBarrelFirstDart.bottomBarrelSecondDartPitch,
        offsetFromBottomBarrelFirstDart.bottomBarrelSecondDartYaw
    )
    { }

    void EntireDartCodeBaseCommand::initialize()
    {
        barrel = Barrel::NONE;
        dart = Dart::NONE;
        currentPitchPositionInSteps = turret.pitchMotor.getCurrentPosition();
        currentYawPositionInSteps = turret.yawMotor.getCurrentPosition();
        bottomBarrelFirstDartPitchPositionInSteps = currentPitchPositionInSteps;
        bottomBarrelFirstDartPitchPositionInSteps = currentYawPositionInSteps;
    }

    void EntireDartCodeBaseCommand::updateStatesFromRemote(aruwsrc::Drivers* drivers)
    {
        tap::communication::serial::Remote::SwitchState leftSwitchState
            = drivers->remote.getSwitch(tap::communication::serial::Remote::Switch::LEFT_SWITCH);
        tap::communication::serial::Remote::SwitchState rightSwitchState
            = drivers->remote.getSwitch(tap::communication::serial::Remote::Switch::RIGHT_SWITCH);

        if (rightSwitchState == SWITCH_UP)
        {
            barrel = Barrel::TOP;
        }
        else if (rightSwitchState == SWITCH_MID)
        {
            barrel = Barrel::BOTTOM;
        }
        else
        {
            barrel = Barrel::NONE;
        }

        if (leftSwitchState == SWITCH_UP)
        {
            dart = Dart::SECOND;
        }
        else if (leftSwitchState == SWITCH_MID)
        {
            dart = Dart::FIRST;
        }
        else
        {
            dart = Dart::NONE;
        }
    }

    void EntireDartCodeBaseCommand::execute()
    {
        updateStatesFromRemote(drivers);
        currentPitchPositionInSteps += turret.pitchMotor.getCurrentPosition() - currentPitchPositionInSteps;
        currentYawPositionInSteps += turret.yawMotor.getCurrentPosition() - currentYawPositionInSteps;

        if (barrel == Barrel::NONE)
        {
            
        }

        comprisedCommandScheduler.run();
    }
}