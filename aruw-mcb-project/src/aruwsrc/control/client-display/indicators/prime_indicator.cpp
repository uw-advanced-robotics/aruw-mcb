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

#include "prime_indicator.hpp"

#include "tap/drivers.hpp"

using namespace tap::communication::serial;

namespace aruwsrc::control::client_display
{
PrimeIndicator::PrimeIndicator(RefSerialTransmitter &refSerialTransmitter, const RefSerial &refSerial)
    : HudIndicator(refSerialTransmitter),
      numberIndicator(refSerialTransmitter, &numberGraphic, updatePrimeDisplay, (int32_t)0),
      currentPrime(2),
      currentComputation(2),
      refSerial(refSerial)
{
}

modm::ResumableResult<void> PrimeIndicator::update()
{
    // // Access the correct field depending on the robot type
    // if (refSerial.getRobotData().robotId == RefSerialData::RobotId::BLUE_HERO ||
    //     refSerial.getRobotData().robotId == RefSerialData::RobotId::RED_HERO)
    // {
    //     bulletCount = static_cast<int16_t>(refSerial.getRobotData().turret.bulletsRemaining42);
    // }
    // else
    // {
    //     bulletCount = static_cast<int16_t>(refSerial.getRobotData().turret.bulletsRemaining17);
    // }

    currentComputation++;

    int isPrime = 1;
    // check if currentComputation is prime
    for (int i = 2; i * i <= currentComputation; ++i) {
        if ((currentComputation % i) == 0) {
            isPrime = 0;
            break;
        }
    }

    if (isPrime) {
        currentPrime = currentComputation;
    }

    RF_BEGIN(1);

    numberIndicator.setIndicatorState(currentPrime);
    RF_CALL(numberIndicator.draw());

    RF_END();
}

modm::ResumableResult<void> PrimeIndicator::sendInitialGraphics()
{
    RF_BEGIN(0);

    RF_CALL(refSerialTransmitter.sendGraphic(&textGraphic));

    RF_CALL(numberIndicator.initialize());

    RF_END();
}

void PrimeIndicator::initialize()
{
    uint8_t graphicName[3];

    getUnusedGraphicName(graphicName);
    RefSerialTransmitter::configGraphicGenerics(
        &textGraphic.graphicData,
        graphicName,
        Tx::GRAPHIC_ADD,
        DEFAULT_GRAPHIC_LAYER,
        Tx::GraphicColor::YELLOW);

    RefSerialTransmitter::configCharacterMsg(SIZE, WIDTH, TEXT_X, TEXT_Y, "PRIME: ", &textGraphic);

    getUnusedGraphicName(graphicName);
    RefSerialTransmitter::configGraphicGenerics(
        &numberGraphic.graphicData,
        graphicName,
        Tx::GRAPHIC_ADD,
        DEFAULT_GRAPHIC_LAYER,
        Tx::GraphicColor::YELLOW);

    updatePrimeDisplay(currentPrime, &numberGraphic);
}

}  // namespace aruwsrc::control::client_display
