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
#ifndef LAUNCHER_SUBSYSTEM_HPP_
#define LAUNCHER_SUBSYSTEM_HPP_

#include "tap/control/subsystem.hpp"
#include <tap/motor/dji_motor.hpp>
class Drivers;
namespace dart::subsystem
{

class DartLauncherSubsystem : public tap::control::Subsystem
{
public: 
    DartLauncherSubsystem(
        tap::Drivers &drivers, tap::motor::MotorInterface &pullMotor);

    void initialize() override;

    void refresh() override;

    void refreshSafeDisconnect() override;

    const char* getName() const override { return "Dart_Launcher_Subsystem"; }
protected:
    tap::motor::MotorInterface &motor;

};  // class DartLauncherSubsystem


}  // dart::subsystem
#endif  // LAUNCHER_SUBSYSTEM_HPP_
