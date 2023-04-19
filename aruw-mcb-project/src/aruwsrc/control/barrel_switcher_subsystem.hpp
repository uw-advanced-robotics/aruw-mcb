/*
 * Copyright (c) 2022-2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef BARREL_SWITCHER_SUBSYSTEM_HPP
#define BARREL_SWITCHER_SUBSYSTEM_HPP

#include "aruwsrc/control/homeable_subsystem_interface.hpp"

#include "tap/drivers.hpp"
#include "tap/motor/dji_motor.hpp"

namespace aruwsrc::control
{

// static constexpr int32_t HOMING_MOTOR_OUTPUT = SHRT_MAX / 2;


static constexpr int32_t USING_RIGHT_BARREL_POSITION = 25; //find actual value after hardware testing
static constexpr int32_t USING_LEFT_BARREL_POSITION = 75; //find actual value after hardware testing

//find actual values after testing
static constexpr float POSITION_PID_KP = 1.0f;
static constexpr float POSITION_PID_KI = 1.0f;
static constexpr float POSITION_PID_KD = 1.0f;
static constexpr int32_t POSITION_PID_MAX_ERROR_SUM = 1;
static constexpr int32_t POSITION_PID_MAX_OUTPUT = 1;

enum class FiringPosition
    {
        USING_LEFT_BARREL,
        USING_RIGHT_BARREL,
        SWITCHING_BETWEEN_BARRELS
    };

class BarrelSwitcherSubsystem : public aruwsrc::control::HomeableSubsystemInterface
{
public:
    BarrelSwitcherSubsystem(
        tap::Drivers* drivers, 
        tap::motor::MotorId motorid, aruwsrc::control::HomingConfig config
    );
    
    void initialize() override;
    void refresh() override;
    int32_t getHomingMotorOutput() override;
    bool isStalled() const override;
    void setLowerBound() override;
    void setUpperBound() override;
    void moveTowardUpperBound() override;
    void moveTowardLowerBound() override; 
    void stop() override;
    
private:
    void setMotorVelocity(int32_t velocity);
    void updateMotorEncoderPid(
        modm::Pid<int32_t>* pid,
        tap::motor::DjiMotor* const motor,
        int32_t desiredEncoderPosition
    );

    /**
     * The motor that switches the turret's barrels
    */
   tap::motor::DjiMotor motor;

    /**
     * upper bound for motor's encoder
     * note: the lower bound is 0
    */
    int32_t motorUpperBound;

    /**
     * stores the thresholds for shaftRPM and torque; used to indicate motor stall
    */
    aruwsrc::control::HomingConfig config;

    bool lowerBoundSet;
    bool upperBoundSet;

    /**
     * Stores the motor's position along the axis 
    */
    int32_t motorPosition;

    /**
     * Stores the state of this barrel switcher's firing position; which barrel is in use
    */
    FiringPosition firingPosition;

    modm::Pid<int32_t>* encoderPid;
};
} //namespace aruwsrc::control
#endif