/*
 * Copyright (c) 2026-2026 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "engineer_text_indicators.hpp"

using namespace tap::communication::serial;

namespace aruwsrc::control::client_display::indicators
{
EngineerTextIndicators::EngineerTextIndicators(
    tap::Drivers &drivers,
    const aruwsrc::control::imu::ImuCalibrateCommand &imuCalibrateCommand,
    const tap::communication::sensors::limit_switch::LimitSwitchInterface &wristPressureSensor,
    const tap::communication::sensors::limit_switch::LimitSwitchInterface
        &cubeStoragePressureSensor1,
    const tap::communication::sensors::limit_switch::LimitSwitchInterface
        &cubeStoragePressureSensor2,
    tap::communication::serial::RefSerialTransmitter &refSerialTransmitter)
    : HudIndicator(refSerialTransmitter),
      drivers(drivers),
      imuCalibrateCommand(imuCalibrateCommand),
      wristPressureSensor(wristPressureSensor),
      cubeStoragePressureSensor1(cubeStoragePressureSensor1),
      cubeStoragePressureSensor2(cubeStoragePressureSensor2)
{
}

modm::ResumableResult<void> EngineerTextIndicators::update()
{
    RF_BEGIN(1);

    memcpy(prevStates, states, sizeof(states));

    // Update states
    states[IMU_CALIBRATING] = drivers.commandScheduler.isCommandScheduled(&imuCalibrateCommand);
    states[CUBE_HELD] = wristPressureSensor.getLimitSwitchDepressed();
    states[CUBE_STORAGE_1] = cubeStoragePressureSensor1.getLimitSwitchDepressed();
    states[CUBE_STORAGE_2] = cubeStoragePressureSensor2.getLimitSwitchDepressed();

    for (index = 0; index < NUM_TEXT_HUD_INDICATORS; index++)
    {
        // If the state has changed, update the graphic
        if (prevStates[index] != states[index])
        {
            textHudIndicatorGraphics[index].graphicData.operation =
                states[index] ? Tx::GRAPHIC_ADD : Tx::GRAPHIC_DELETE;
            RF_CALL(refSerialTransmitter.sendGraphic(&textHudIndicatorGraphics[index]));
        }
    }

    RF_END();
}

void EngineerTextIndicators::initialize()
{
    uint8_t graphicName[3];
    for (int i = 0; i < NUM_TEXT_HUD_INDICATORS; i++)
    {
        getUnusedGraphicName(graphicName);

        const TextIndicatorData &textIndicator = INDICATOR_LIST[i];

        RefSerialTransmitter::configGraphicGenerics(
            &textHudIndicatorGraphics[i].graphicData,
            graphicName,
            Tx::GRAPHIC_ADD,
            DEFAULT_GRAPHIC_LAYER,
            textIndicator.color);

        RefSerialTransmitter::configCharacterMsg(
            textIndicator.size,
            textIndicator.textWidth,
            textIndicator.x,
            textIndicator.y,
            textIndicator.text,
            &textHudIndicatorGraphics[i]);
    }
}

}  // namespace aruwsrc::control::client_display::indicators
