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
#ifndef DART_RELOADER_SUBSYSTEM_HPP_
#define DART_RELOADER_SUBSYSTEM_HPP_

#include "tap/control/subsystem.hpp"
#include <tap/motor/dji_motor.hpp>
#include "tap/algorithms/smooth_pid.hpp"

namespace aruwsrc::robot::dart
{

class DartReloaderSubsystem : public tap::control::Subsystem
{
public: 
    DartReloaderSubsystem(
        tap::Drivers* drivers,
        tap::motor::MotorInterface& motor
    );

    void initialize() override;

    void setSetpoint(float32_t setpoint) ;
    bool atSetpoint() ;
    float32_t getPostion() const { return motor.getEncoder()->getPosition().getUnwrappedValue(); }
    void refresh() override;
    float32_t getSetpoint() const { return setpoint; }
    void refreshSafeDisconnect() override;

    const char* getName() const override { return "Dart Reloader Subsystem"; }
    float currentPosition = 0.0f;
private:
    tap::motor::MotorInterface &motor;
    tap::algorithms::SmoothPid pidController;
    float32_t setpoint = 0;
};  // class DartReloaderSubsystem

}  // namespace aruwsrc::robot::dart
#endif  // DART_RELOADER_SUBSYSTEM_HPP_
