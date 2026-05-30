/*
 * Copyright (c) 2021-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "engineer_turret_subsystem.hpp"

#include "aruwsrc/communication/can/turret_mcb_can_comm.hpp"

namespace aruwsrc::engineer
{
void EngineerTurretSubsystem::refresh()
{
    control::turret::RobotTurretSubsystem::refresh();
    updatePitchLimitDebug();
}

void EngineerTurretSubsystem::refreshSafeDisconnect()
{
    control::turret::RobotTurretSubsystem::refreshSafeDisconnect();
    DEBUG_currentPitchAngle = getWorldPitch();
    updatePitchLimitDebug();
}

void EngineerTurretSubsystem::updatePitchLimitDebug()
{
    DEBUG_livePitchMinLimit = pitchMotor.getMinLimit();
    DEBUG_livePitchMaxLimit = pitchMotor.getMaxLimit();
}

float EngineerTurretSubsystem::getWorldYaw() const { return getIMU()->getYaw(); }

float EngineerTurretSubsystem::getWorldPitch() const { return getIMU()->getPitch(); }

}  // namespace aruwsrc::engineer
