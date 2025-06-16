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

 #include "enemy_indicator.hpp"

 #include "tap/drivers.hpp"
 #include "tap/communication/serial/ref_serial.hpp"
 #include "tap/communication/serial/ref_serial_data.hpp"
 
 using namespace tap::communication::serial;
 
 namespace aruwsrc::control::client_display
 {
 EnemyIndicator::EnemyIndicator(RefSerialTransmitter &refSerialTransmitter, const RefSerial &refSerial)
     : HudIndicator(refSerialTransmitter),
       refSerial(refSerial)
       
 {
 }
 
 modm::ResumableResult<void> EnemyIndicator::sendInitialGraphics()
 {
     RF_BEGIN(0);
     
     RF_CALL(refSerialTransmitter.sendGraphic(&enemyGraphic));
     
     RF_END();
 }
 
 
 void EnemyIndicator::initialize()
 {
     uint8_t graphicName[3];

     RefSerialData::RobotId ourRobot = refSerial.getRobotData().robotId;
     bool isBlue = RefSerialData::isBlueTeam(ourRobot);
     
     if (isBlue) {
        textX = 123; // left side cuz enemy is red
     } else {
        textX = 1574; // right side cuz enemy is blue
     }
     
     getUnusedGraphicName(graphicName);
     RefSerialTransmitter::configGraphicGenerics(
         &enemyGraphic.graphicData,
         graphicName,
         Tx::GRAPHIC_ADD,
         DEFAULT_GRAPHIC_LAYER,
         Tx::GraphicColor::YELLOW);
 
     RefSerialTransmitter::configCharacterMsg(SIZE, WIDTH, textX, TEXT_Y, "ENEMY", &enemyGraphic);
     

 }
 
 }  // namespace aruwsrc::control::client_display