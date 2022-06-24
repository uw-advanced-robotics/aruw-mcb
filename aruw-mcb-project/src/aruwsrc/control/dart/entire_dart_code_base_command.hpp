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

#ifndef ENTIRE_DART_CODE_BASE_COMMAND_HPP_
#define ENTIRE_DART_CODE_BASE_COMMAND_HPP_

#include "aruwsrc/drivers.hpp"
#include "tap/control/comprised_command.hpp"
#include "tap/control/setpoint/commands/move_absolute_command.hpp"

#include "aruwsrc/control/turret/stepper_turret_subsystem.hpp"
#include "aruwsrc/control/turret/user/stepper_motor_turret_control_command.hpp"
#include "aruwsrc/control/agitator/agitator_subsystem.hpp"

namespace aruwsrc::control::dart 
{
struct ConfigAConfig
{
    float dartOneAgitatorSetpoint;  // Radians
    float dartTwoAgitatorSetpoint;  // Radians
    float agitatorAngularSpeed;  // Radians/Second
    float agitatorSetpointTolerance;  // Radians
    float timeDelayAfterLaunching; // Milliseconds
};
struct OffsetFromBottomBarrelFirstDart
{
    float bottomBarrelSecondDartYaw;  // Radians
    float bottomBarrelSecondDartPitch;  // Radians
    float topBarrelFirstDartYaw;  // Radians
    float topBarrelFirstDartPitch;  // Radians
    float topBarrelSecondDartYaw;  // Radians
    float topBarrelSecondDartPitch;  // Radians
};

/**
 * Comprised Command which launches two darts on the dart launcher
 * from a single barrel, with a parameter for any positional offset needed
 * between darts and a constant wait time between the first launch
 * and moving to the offset position for the second launch.
 * 
 * This command required a `StepperTurretSubsystem` for the dart pitch & yaw,
 * and an `AgitatorSubsystem` for the barrel sled.
 */
class EntireDartCodeBaseCommand : public tap::control::ComprisedCommand
{
public:
    /**
     * @brief Construct a new Launch Darts Comprised Command object
     * with an offset for the next dart.
     * 
     * @param[in] drivers: A reference to the `drivers` singleton instance.
     * @param[in] turret: A reference to the `StepperTurretSubsystem`
     * for the pitch & yaw of the launcher.
     * @param[in] sled: A reference to the `AgitatorSubsystem`
     * for the barrel sled.
     * @param[in] config: The configuration parameters for this command,
     * as listed in the `LaunchDartComprisedCommand` struct.
     * @param[in] offsetConfig: The configuration for moving an offset after launching,
     * as listed in the `LaunchDartOffsetConifg` struct.
     * If the default value is passed, the command will end
     * immediately after launching the dart.
     */
    EntireDartCodeBaseCommand(
        aruwsrc::Drivers *drivers,
        aruwsrc::control::turret::StepperTurretSubsystem& turret,
        aruwsrc::agitator::AgitatorSubsystem& bottomBarrelSled,
        aruwsrc::agitator::AgitatorSubsystem& topBarrelSled,
        ConfigAConfig configA,
        OffsetFromBottomBarrelFirstDart offsetFromBottomBarrelFirstDart
    );

    const char *getName() const override { return "launch dart command"; }
    void initialize() override;
    void execute() override;
    void end(bool interrupted) override;
    bool isFinished() const override;

private:
    aruwsrc::Drivers* drivers;
    aruwsrc::control::turret::StepperTurretSubsystem& turret;
    aruwsrc::agitator::AgitatorSubsystem& bottomBarrelSled;
    aruwsrc::agitator::AgitatorSubsystem& topBarrelSled;

    float TIME_DELAY_AFTER_LAUNCHING;

    int bottomBarrelFirstDartPitchPositionInSteps;
    int bottomBarrelFirstDartYawPositionInSteps;
    int currentPitchPositionInSteps;
    int currentYawPositionInSteps;

    float prevTime;

    bool turretIsMoving;
    enum Barrel
    {
        TOP,
        BOTTOM,
        NONE
    };
    Barrel barrel;

    enum Dart
    {
        FIRST,
        SECOND,
        NONE
    };
    Dart dart;

    tap::communication::serial::Remote::SwitchState SWITCH_UP = tap::communication::serial::Remote::SwitchState::UP;
    tap::communication::serial::Remote::SwitchState SWITCH_MID = tap::communication::serial::Remote::SwitchState::MID;
    tap::communication::serial::Remote::SwitchState SWITCH_DOWN = tap::communication::serial::Remote::SwitchState::DOWN;
    tap::communication::serial::Remote::SwitchState SWITCH_UNKNOWN = tap::communication::serial::Remote::SwitchState::UNKNOWN;

    void updateStatesFromRemote(aruwsrc::Drivers* drivers);

    tap::control::setpoint::MoveAbsoluteCommand launchBottomBarrelFirstDart;
    tap::control::setpoint::MoveAbsoluteCommand launchBottomBarrelSecondDart;
    tap::control::setpoint::MoveAbsoluteCommand launchTopBarrelFirstDart;
    tap::control::setpoint::MoveAbsoluteCommand launchTopBarrelSecondDart;

    aruwsrc::control::turret::user::OffsetStepperMotorTurretControlCommand bottomBarrelSecondDartPositionOffset;
    aruwsrc::control::turret::user::OffsetStepperMotorTurretControlCommand topBarrelFirstDartPositionOffset;
    aruwsrc::control::turret::user::OffsetStepperMotorTurretControlCommand topBarrelSecondDartPositionOffset;
};
}

#endif  // ENTIRE_DART_CODE_BASE_COMMAND_HPP_