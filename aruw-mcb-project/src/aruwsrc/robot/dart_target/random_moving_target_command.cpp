/*
 * Copyright (c) 2023-2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "random_moving_target_command.hpp"
#include "dart_target_constants.hpp"
#include "modm/platform/random/random_number_generator.hpp"

RandomMovingTargetCommand::RandomMovingTargetCommand(
    MotorSubsystem* subsystem)
    : motorSubsystem(subsystem)
{
    this->addSubsystemRequirement(subsystem);
}

void RandomMovingTargetCommand::initialize()
{
    targetSet = false;
    startTime = tap::arch::clock::getTimeMilliseconds();
}

void RandomMovingTargetCommand::execute()
{
    if (!targetSet && modm::platform::RandomNumberGenerator::isReady()) {
        targetPos = modm::platform::RandomNumberGenerator::getValue() % aruwsrc::dart_target::constants::TARGET_TRAVEL_DISTANCE;
    }

    if (targetSet) {
        motorSubsystem->setDesiredRPM(aruwsrc::dart_target::constants::TARGET_MOVEMENT_SPEED);
    }
}

void RandomMovingTargetCommand::end(bool) { motorSubsystem->stop(); }

bool RandomMovingTargetCommand::isFinished() const {
    return (tap::arch::clock::getTimeMilliseconds() - startTime) >= aruwsrc::dart_target::constants::TARGET_TIMEOUT; // if it times out, just time out and stop moving
}
