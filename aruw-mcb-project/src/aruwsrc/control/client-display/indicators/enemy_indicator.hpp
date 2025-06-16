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

 #ifndef ENEMY_INDICATOR_HPP_
 #define ENEMY_INDICATOR_HPP_
 
 #include "tap/communication/referee/state_hud_indicator.hpp"
 #include "tap/communication/serial/ref_serial.hpp"
 
 #include "modm/processing/resumable.hpp"
 
 #include "hud_indicator.hpp"
 
 using namespace tap::communication::serial;
 
 namespace aruwsrc::control::client_display
 {
 /**
  * Adds text to show "ENEMY" in bright yellow under the enemy team's side.
  * Displays ENEMY under red side if we're blue team, and vice versa.
  */
 class EnemyIndicator : public HudIndicator, protected modm::Resumable<2>
 {
 public:
     /**
      * Construct a EnemyIndicator object.
      *
      * @param[in] refSerialTransmitter RefSerialTransmitter instance.
      */
     EnemyIndicator(
         tap::communication::serial::RefSerialTransmitter &refSerialTransmitter,
         const tap::communication::serial::RefSerial &refSerial);
 
     void initialize() override final;
 
     modm::ResumableResult<void> sendInitialGraphics() override final;
 
     //modm::ResumableResult<void> update() override final;
 
 private:
     // X position of the text (will be set based on team)
     uint16_t textX;
     // Y position of the text
     static constexpr uint16_t TEXT_Y = 865; 
     // WIDTH of the text
     static constexpr uint16_t WIDTH = 4;
     // SIZE of the text
     static constexpr uint16_t SIZE = 40;
 
     Tx::GraphicCharacterMessage enemyGraphic;
 
     const tap::communication::serial::RefSerial &refSerial;
    
 };
 
 }  // namespace aruwsrc::control::client_display
 
 #endif  // ENEMY_INDICATOR_HPP_