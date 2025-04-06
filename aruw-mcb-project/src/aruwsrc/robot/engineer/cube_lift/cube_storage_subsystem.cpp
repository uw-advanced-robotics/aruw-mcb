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

#include "cube_storage_subsystem.hpp"
#include "tap/motor/dji_motor.hpp"

namespace aruwsrc::robot::engineer
{
CubeStorageSubsystem::CubeStorageSubsystem(
    tap::Drivers* drivers,
    tap::motor::MotorInterface& storageLiftMotor, aruwsrc::control::TriggerInterface& trigger,
    uint64_t length)
    : OneSidedBoundedSubsystemInterface(drivers, trigger, length),
      motor(storageLiftMotor){};

void CubeStorageSubsystem::initialize() { motor.initialize(); moveMotor(0); }

void CubeStorageSubsystem::moveMotor(int16_t power) { motor.setDesiredOutput(power + FEEDFORWARD); }

void CubeStorageSubsystem::setSetpoint(float newSetpoint) {
    setpoint = newSetpoint;
}

float CubeStorageSubsystem::getSetpoint() {
    return setpoint;
}

void CubeStorageSubsystem::refreshSafeDisconnect() { motor.setDesiredOutput(0); }

bool CubeStorageSubsystem::isLimitSwitched() { return drivers->digital.read(CUBELIFT_LIMITSWITCH_PORT); }

void CubeStorageSubsystem::refresh() { 
    limit = isLimitSwitched(); 
    float error = setpoint - motor.getPositionUnwrapped() / tap::motor::DjiMotor::GEAR_RATIO_M3508 * MM_PER_REVOLUTION;
    float errorDerivative = motor.getShaftRPM() / 1000 / 60 / tap::motor::DjiMotor::GEAR_RATIO_M3508 * MM_PER_REVOLUTION / 1000;
    float timeDifference = (tap::arch::clock::getTimeMilliseconds() - lastTime) / 1000;
    lastTime = tap::arch::clock::getTimeMilliseconds(); 
    pid.runController(error, errorDerivative, timeDifference);
    motor.setDesiredOutput(pid.getOutput() + FEEDFORWARD);
}
}  // namespace aruwsrc::robot::engineer
