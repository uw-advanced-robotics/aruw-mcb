/*
 * Copyright (c) 2020-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef STEPPER_TURRET_SUBSYSTEM_HPP_
#define STEPPER_TURRET_SUBSYSTEM_HPP_

#include "tap/control/subsystem.hpp"
#include "tap/motor/stepper_motor_interface.hpp"
#include "tap/control/turret_subsystem_interface.hpp"

#include "turret_motor_config.hpp"

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
#include "aruwsrc/mock/turret_motor_mock.hpp"
#else
#include "turret_motor.hpp"
#endif

namespace aruwsrc
{
class Drivers;
}

namespace aruwsrc::control::turret
{
class StepperTurretSubsystem : public tap::control::Subsystem
{
/**
 * Stores software necessary for interacting with stepper motors.
 */
public:
   /**
     * Constructs a StepperTurretSubsystem.
     *
     * @param[in] drivers Pointer to a drivers singleton object.
     * @param[in] pitchMotor Pointer to pitch motor that this `StepperTurretSubsystem` will own.
     * @param[in] yawMotor Pointer to yaw motor that this `StepperTurretSubsystem` will own.
     */
    explicit StepperTurretSubsystem(
        aruwsrc::Drivers* drivers,
        tap::motor::StepperMotorInterface& pitchMotor,
        tap::motor::StepperMotorInterface& yawMotor);
            
    void initialize() override;

    void refresh() override;

    const char* getName() override { return "Turret"; }

    void onHardwareTestStart() override;

    mockable inline bool isOnline() const { return true;}; // JENNY_TODO : check if this is right...coz i deleted the TurretMotors

public:
    // Associated with and contains logic for controlling the stepper motor pitch.
    tap::motor::StepperMotorInterface& pitchMotor;
    // Associated with and contains logic for controlling the stepper motor yaw.
    tap::motor::StepperMotorInterface& yawMotor;

protected:
    Drivers* drivers;
};

} // namespace aruwsrc::control::turret

#endif // STEPPER_TURRET_SUBSYSTEM_HPP_
