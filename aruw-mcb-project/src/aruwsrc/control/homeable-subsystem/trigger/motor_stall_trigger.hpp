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

#ifndef MOTOR_STALL_TRIGGER_HPP_
#define MOTOR_STALL_TRIGGER_HPP_

#include "tap/motor/dji_motor.hpp"

#include "aruwsrc/control/homeable-subsystem/trigger/trigger_interface.hpp"

/**
 * Represents a "trigger" used by Homeable Subsystems to detect
 * through the stalling of the motor when it is at an end of its axis.
 */
class MotorStallTrigger : TriggerInterface
{
public:
    MotorStallTrigger(tap::motor::DjiMotor& motor, int16_t maxRPM, int16_t minTorque);

    /**
     * Detects whether the subsystem's motor is stalled, indicating that the trigger is triggered.
     */
    bool isTriggered();

private:
    tap::motor::DjiMotor* motor;
    int16_t maxRPM;
    int16_t minTorque;
};

#endif
