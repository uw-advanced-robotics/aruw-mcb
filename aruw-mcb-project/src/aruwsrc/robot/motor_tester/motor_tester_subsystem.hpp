/*
* Copyright (c) 2025-2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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
#ifndef MOTOR_TESTER_SUBSYSTEM_HPP_
#define MOTOR_TESTER_SUBSYSTEM_HPP_

#include "tap/control/subsystem.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/motor/double_dji_motor.hpp"

namespace aruwsrc::motor_tester
{

class MotorSubsystem : public tap::control::Subsystem
{
public: 
    inline MotorSubsystem(
        tap::Drivers* drivers, tap::motor::MotorInterface &motor)
        : Subsystem(drivers), motor(motor){};

    void initialize() override {motor.initialize();}
    bool isMotorOnline() {
        return motor.isMotorOnline();
    }

    int getDesiredOutput(){
        return desiredOutput;
    }
    void setDesiredOutput(float output){
        desiredOutput = std::clamp<int32_t>(output, -tap::motor::DjiMotor::MAX_OUTPUT_C620, tap::motor::DjiMotor::MAX_OUTPUT_C620);
    }

    tap::algorithms::WrappedFloat getMotorPosition(){
        return motor.getEncoder()->getPosition();
    }
    float getMotorVelocity(){
        return motor.getEncoder()->getVelocity();
    }

    void refreshSafeDisconnect() {
        motor.setDesiredOutput(0);
    }

    void refresh(){
        motor.setDesiredOutput(desiredOutput);
    }

private:
    tap::motor::MotorInterface &motor;
    float desiredOutput;
};  // class CLASS_NAME

}  // NAMESPACE
#endif  // MOTOR_TESTER_SUBSYSTEM_HPP_
