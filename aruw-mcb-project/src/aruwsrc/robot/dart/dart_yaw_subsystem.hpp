/*
* Copyright (c) 2024-2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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
#ifndef DART_YAW_SUBSYSTEM_HPP_
#define DART_YAW_SUBSYSTEM_HPP_

#include "tap/control/subsystem.hpp"
#include <tap/motor/dji_motor.hpp>
#include "tap/algorithms/smooth_pid.hpp"
#include "dart_constants.hpp"


namespace dart::dart_yaw_subsystem
{

class DartYawSubsystem : public tap::control::Subsystem
{
public: 
    DartYawSubsystem(
        tap::Drivers &drivers, tap::motor::MotorInterface &deadMotor, tap::motor::MotorInterface &yawMotor);

    void initialize() override;
    void setSetpoint(float setpoint);
    bool getDigitalPin();
    void reset();

    void refresh() override;

    void refreshSafeDisconnect() override;

    const char* getName() const override { return "DartYawSubsystem"; }

    protected:
        tap::motor::MotorInterface &deadMotor;
        tap::motor::MotorInterface &yawMotor;

    private: 
        tap::algorithms::SmoothPid pid = tap::algorithms::SmoothPid(aruwsrc::control::turret::YAW_MOTOR_PID_CONFIG);
        float setpoint;
        float lastTime;
        tap::gpio::Digital &digital;



};  // class DartYawSubsystem

}  // NAMESPACE
#endif  // DART_YAW_SUBSYSTEM_HPP_
