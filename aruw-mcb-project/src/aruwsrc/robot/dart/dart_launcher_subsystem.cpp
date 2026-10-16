/*
 * Copyright (c) 2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "dart_launcher_subsystem.hpp"

#include "aruwsrc/control/turret/constants/turret_constants.hpp"

#include "dart_constants.hpp"
#include "dart_drivers.hpp"
using namespace aruwsrc::control::turret;

namespace aruwsrc::dart
{
DartLauncherSubsystem::DartLauncherSubsystem(
    tap::Drivers* drivers,
    tap::motor::MotorInterface& pullMotors)
    : Subsystem(drivers),
      pullMotors(pullMotors),
      servo(drivers, SERVO_PORT, SERVO_MAX, SERVO_MIN, SERVO_SPEED)
{
    servo.setTargetPwm(SERVO_MAX);
};

void DartLauncherSubsystem::initialize()
{
    drivers->pwm.setTimerFrequency(tap::gpio::Pwm::TIMER8, 500);
    pullMotors.initialize();
}

void DartLauncherSubsystem::moveMotor(int32_t power) { pullMotors.setDesiredOutput(power); }

bool DartLauncherSubsystem::isBeamBroken() { return beamBroken; }

bool DartLauncherSubsystem::isLimitSwitched() { return drivers->digital.read(LIMITSWITCH_PORT); }

void DartLauncherSubsystem::refresh() { beamBroken = !drivers->digital.read(BEAMBREAK_PORT); }

void DartLauncherSubsystem::refreshSafeDisconnect() { pullMotors.setDesiredOutput(0); }

void DartLauncherSubsystem::setOpen() { servo.setTargetPwm(servo.getMaxPWM()); }

void DartLauncherSubsystem::setClose() { servo.setTargetPwm(servo.getMinPWM()); }

float DartLauncherSubsystem::getOpenPWM() { return servo.getMaxPWM(); }

float DartLauncherSubsystem::getClosePWM() { return servo.getMinPWM(); }

}  // namespace aruwsrc::dart