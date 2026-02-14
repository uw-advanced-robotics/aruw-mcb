/*
 * Copyright (c) 2020-2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "auto_nav_command.hpp"

using namespace tap::algorithms;
using namespace tap::communication::sensors::imu::mpu6500;
using namespace tap::communication::serial;

using GameType = RefSerialData::Rx::GameType;
using GameStage = RefSerialData::Rx::GameStage;
using GameData = RefSerialData::Rx::GameData;

namespace aruwsrc
{
namespace control::chassis
{
AutoNavCommand::AutoNavCommand(
    const tap::Drivers& drivers,
    chassis::HolonomicChassisSubsystem& chassis,
    aruwsrc::control::chassis::ChassisAutoNavController& autoNavController,
    bool autoNavOnlyInGame, bool beybladeEnabled, bool ends)
    : drivers(drivers),
      chassis(chassis),
      autoNavController(autoNavController),
      autoNavOnlyInGame(autoNavOnlyInGame),
      beybladeEnabled(beybladeEnabled),
      ends(ends)
{
    // TODO: sucks that we have to pull the address out of the reference bc everything else uses
    // pointers
    addSubsystemRequirement(&chassis);
}

void AutoNavCommand::initialize() { 
    autoNavController.initialize();
}

void AutoNavCommand::execute()
{
    const float maxWheelSpeed = chassis::HolonomicChassisSubsystem::getMaxWheelSpeed(
        drivers.refSerial.getRefSerialReceivingData(),
        drivers.refSerial.getRobotData().chassis.powerConsumptionLimit);

    const GameData gameData = drivers.refSerial.getGameData();

    if (!autoNavOnlyInGame ||
        (gameData.gameType == GameType::UNKNOWN || (gameData.gameStage == GameStage::IN_GAME)))
    {
        autoNavController.runController(maxWheelSpeed, movementEnabled, beybladeEnabled);
    }
    else
    {
        chassis.setDesiredOutput(0., 0., 0.);
    }
}

void AutoNavCommand::end(bool) { chassis.setZeroRPM(); }
bool AutoNavCommand::isFinished() const {
    return ends ? autoNavController.atSetpoint() : false; 
}
}

  // namespace control::chassis::sentry

}  // namespace aruwsrc
