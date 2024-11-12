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
    // Either the agitator is online and not jammed, or the shooter has no power
    if ((agitatorSubsystem.isOnline() && !agitatorSubsystem.isJammed()) ||
        !(drivers.refSerial.getRobotData().robotPower & Rx::RobotPower::SHOOTER_HAS_POWER))
    {
        jamTimeout.restart(JAM_TIMEOUT_MS);
    }

    memcpy(prevStates, states, sizeof(states));

    // Defined outside due to RF
    states[AGITATOR_JAMMED] = jamTimeout.isExpired();
    states[IMU_CALIBRATING] = drivers.commandScheduler.isCommandScheduled(&imuCalibrateCommand);
    states[NOT_SPINNING] = true;

    // Check if either of the commands are running
    for (auto command : validChassisCommands)
    {
        states[NOT_SPINNING] &= !drivers.commandScheduler.isCommandScheduled(command);
    }

    // Check if we are actually in a match
    states[NOT_SPINNING] &= drivers.refSerial.getGameData().gameStage != Rx::GameStage::PREMATCH;

    RF_BEGIN(0);

    for (index = 0; index < NUM_TEXT_HUD_INDICATORS; index++)
    {
        if (prevStates[index] != states[index])
        {
            textHudIndicatorGraphics[index].graphicData.operation =
                states[index] ? Tx::GRAPHIC_ADD : Tx::GRAPHIC_DELETE;
            RF_CALL(refSerialTransmitter.sendGraphic(&textHudIndicatorGraphics[index]));
        }
    }

    RF_END();
}

modm::ResumableResult<bool> TextHudIndicators::sendInitialGraphics()
{
    RF_BEGIN(1);
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
            textIndicator.textWidth,
            textIndicator.x,
            textIndicator.y,
            textIndicator.text,
            &textHudIndicatorGraphics[i]);
    }
}

}  // namespace aruwsrc::control::client_display
