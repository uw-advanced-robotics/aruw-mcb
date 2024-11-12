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

#include "text_hud_indicators.hpp"

using namespace tap::communication::serial;

namespace aruwsrc::control::client_display
{
TextHudIndicators::TextHudIndicators(
    tap::Drivers &drivers,
    tap::control::setpoint::SetpointSubsystem &agitatorSubsystem,
    const aruwsrc::control::imu::ImuCalibrateCommand &imuCalibrateCommand,
    const std::vector<tap::control::Command *> validChassisCommands,
    tap::communication::serial::RefSerialTransmitter &refSerialTransmitter)
    : HudIndicator(refSerialTransmitter),
      drivers(drivers),
      agitatorSubsystem(agitatorSubsystem),
      imuCalibrateCommand(imuCalibrateCommand),
      validChassisCommands(validChassisCommands)
{
}

modm::ResumableResult<bool> TextHudIndicators::update()
{
    // Defined outside due to RF
    int i = 0;

    states[AGITATOR_JAMMED] = agitatorSubsystem.isJammed() || !agitatorSubsystem.isOnline();
    states[IMU_CALIBRATING] = drivers.commandScheduler.isCommandScheduled(&imuCalibrateCommand);
    states[NOT_SPINNING] = true;

    for (auto command : validChassisCommands)
    {
        states[NOT_SPINNING] &= !drivers.commandScheduler.isCommandScheduled(command);
    }

    RF_BEGIN(1);

    for (i = 0; i < NUM_TEXT_HUD_INDICATORS; i++)
    {
        textHudIndicatorGraphics[i].graphicData.operation =
            states[i] ? Tx::GRAPHIC_ADD : Tx::GRAPHIC_DELETE;
        RF_CALL(refSerialTransmitter.sendGraphic(&textHudIndicatorGraphics[i]));
    }

    RF_END();
}

modm::ResumableResult<bool> TextHudIndicators::sendInitialGraphics()
{
    // Defined outside due to RF
    int i = 0;

    RF_BEGIN(0);

    // send all text indicators
    for (i = 0; i < NUM_TEXT_HUD_INDICATORS; i++)
    {
        RF_CALL(refSerialTransmitter.sendGraphic(&textHudIndicatorGraphics[i]));
    }

    RF_END();
}

void TextHudIndicators::initialize()
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
            TEXT_WIDTH,
            textIndicator.x,
            textIndicator.y,
            textIndicator.text,
            &textHudIndicatorGraphics[i]);
    }
}

}  // namespace aruwsrc::control::client_display
