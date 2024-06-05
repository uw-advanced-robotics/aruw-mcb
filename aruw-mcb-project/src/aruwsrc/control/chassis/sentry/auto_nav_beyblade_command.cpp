/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "auto_nav_beyblade_command.hpp"

using namespace tap::algorithms;
using namespace tap::communication::sensors::imu::mpu6500;
using namespace tap::communication::serial;

using GameType = RefSerialData::Rx::GameType;
using GameStage = RefSerialData::Rx::GameStage;
using GameData = RefSerialData::Rx::GameData;

namespace aruwsrc
{
namespace chassis
{
AutoNavBeybladeCommand::AutoNavBeybladeCommand(
    const tap::Drivers& drivers,
    HolonomicChassisSubsystem& chassis,
    const aruwsrc::chassis::ChassisAutoNavController autoNavController,
    bool autoNavOnlyInGame)
    : drivers(drivers),
      chassis(chassis),
      autoNavController(autoNavController),
      autoNavOnlyInGame(autoNavOnlyInGame)
{
    // TODO: sucks that we have to pull the address out of the reference bc everything else uses
    // pointers
    addSubsystemRequirement(&chassis);
}

void AutoNavBeybladeCommand::initialize() { autoNavController.initialize(); }

void AutoNavBeybladeCommand::execute()
{
    const float maxWheelSpeed = HolonomicChassisSubsystem::getMaxWheelSpeed(
        drivers.refSerial.getRefSerialReceivingData(),
        drivers.refSerial.getRobotData().chassis.powerConsumptionLimit);

    const GameData gameData = drivers.refSerial.getGameData();

    if (!autoNavOnlyInGame ||
        (gameData.gameType == GameType::UNKNOWN || (gameData.gameStage == GameStage::IN_GAME)))
    {
        autoNavController.runController(maxWheelSpeed, movementEnabled, beybladeEnabled);
    }
}

void AutoNavBeybladeCommand::end(bool) { chassis.setZeroRPM(); }
}  // namespace chassis

}  // namespace aruwsrc
