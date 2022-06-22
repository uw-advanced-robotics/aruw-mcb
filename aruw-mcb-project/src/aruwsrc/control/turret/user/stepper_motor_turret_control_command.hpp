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

#ifndef STEPPER_MOTOR_TURRET_CONTROL_COMMAND_HPP_
#define STEPPER_MOTOR_TURRET_CONTROL_COMMAND_HPP_

#include "tap/control/command.hpp"

#include "../algorithms/turret_controller_interface.hpp"
#include "../stepper_turret_subsystem.hpp"

namespace aruwsrc
{
class Drivers;
}

namespace aruwsrc::control::turret::user
{
/**
 * Command that takes user input from the `ControlOperatorInterface` to control the pitch and yaw
 * axis of some turret using some passed in yaw and pitch controller upon construction.
 */
class StepperMotorTurretControlCommand : public tap::control::Command
{
public:
    /**
     * Accumulator threshold before we take a step. Must be between 0 and 1.
     */
    static constexpr float STEP_THRESHOLD = 0.8;
    static_assert(0.0f <= STEP_THRESHOLD && STEP_THRESHOLD <= 1.0f);

    /**
     * Turret ID for the purposes of interfacing with control operator interface. ID of 0 gives us
     * the right joystick for control which we want.
     */
    static constexpr int TURRET_ID = 0;

    /**
     * @param[in] drivers Pointer to a global drivers object.
     * @param[in] stepperTurretSubsystem Pointer to the stepper turret to control.
     */
    StepperMotorTurretControlCommand(
        aruwsrc::Drivers* drivers,
        StepperTurretSubsystem& stepperTurretSubsystem);

    bool isReady() override;

    const char* getName() const override { return "Stepper turret control"; }

    void initialize() override;

    void execute() override;

    bool isFinished() const override;

    void end(bool) override;

protected:
    aruwsrc::Drivers* drivers;
    tap::motor::StepperMotorInterface& turretPitchMotor;
    tap::motor::StepperMotorInterface& turretYawMotor;

    /**
     * Used for fine yaw control. Accumulates yaw input over time, when checked and greater than
     * STEP_THRESHOLD then we take a step and this its value is set to 0.
     */
    float yawAccumulator = 0.0f;

    /**
     * Used for fine pitch control. Accumulates yaw input over time, should be set
     * back to 0 when checked and greater than STEP_THRESHOLD
     */
    float pitchAccumulator = 0.0f;
};

/**
 * Stepper motor turret control command but adds an offset to the desired setpoint
 * when scheduled and removes it when descheduled.
 *
 * Originally designed for the ARUW dart system, where the barrel needed to pitch up by
 * a constant offset when using the lower barrel. Used to get around weird command mapping/state
 * restrictions.
 */
class OffsetStepperMotorTurretControlCommand : public StepperMotorTurretControlCommand
{
public:
    /**
     * @param[in] drivers Pointer to a global drivers object.
     * @param[in] stepperTurretSubsystem Pointer to the stepper turret to control.
     * @param[in] pitchOffset the pitch offset to use when this command is scheduled and remove when
     *  this command is descheduled
     * @param[in] yawOffset the yaw offset to use when this command is scheduled and removed thwne
     * this command is unscheduled
     */
    OffsetStepperMotorTurretControlCommand(
        aruwsrc::Drivers* drivers,
        StepperTurretSubsystem& stepperTurretSubsystem,
        int pitchOffset,
        int yawOffset,
        int yawOffsetCorrection,
        int yawOffsetCorrectionChance)
        : StepperMotorTurretControlCommand(drivers, stepperTurretSubsystem),
          pitchOffset(pitchOffset),
          yawOffset(yawOffset),
          yawOffsetCorrection(yawOffsetCorrection),
          yawOffsetCorrectionChance(yawOffsetCorrectionChance),
          offsetCounter(0)

    {
    }

    void initialize() override
    {
        turretPitchMotor.moveSteps(pitchOffset);
        turretYawMotor.moveSteps(yawOffset);
    }

    void end(bool) override
    {
        turretPitchMotor.moveSteps(-pitchOffset);
        if (offsetCounter == 0)
        {
            turretYawMotor.moveSteps(-yawOffset - yawOffsetCorrection);
        }
        else
        {
            turretYawMotor.moveSteps(-yawOffset);
        }
        offsetCounter = (offsetCounter + 1) % yawOffsetCorrectionChance;
    }  // If there is a difference between forward and backward offset, this allows for a janky
       // partial step difference in offsets to compensate

private:
    int pitchOffset;
    int yawOffset;
    int yawOffsetCorrection;
    int offsetCounter;
    int yawOffsetCorrectionChance;
};

}  // namespace aruwsrc::control::turret::user

#endif  // STEPPER_MOTOR_TURRET_CONTROL_COMMAND_HPP_
