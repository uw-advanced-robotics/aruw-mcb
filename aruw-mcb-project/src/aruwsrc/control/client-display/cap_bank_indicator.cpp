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

#include "cap_bank_indicator.hpp"

#include "tap/communication/serial/ref_serial.hpp"
#include "tap/communication/serial/ref_serial_transmitter.hpp"
#include "tap/drivers.hpp"

using namespace tap::communication::serial;

namespace aruwsrc::control::client_display
{
CapBankIndicator::CapBankIndicator(
    tap::Drivers &drivers,
    tap::communication::serial::RefSerialTransmitter &refSerialTransmitter,
    const aruwsrc::communication::sensors::power::ExternalCapacitorBank *capBank)
    : HudIndicator(refSerialTransmitter),
      drivers(drivers),
      capBank(capBank)
{
}

modm::ResumableResult<bool> CapBankIndicator::sendInitialGraphics()
{
    RF_BEGIN(0)

    // remove initial graphics
    RF_CALL(refSerialTransmitter.sendGraphic(&capBankGraphics));
    RF_CALL(refSerialTransmitter.sendGraphic(&capBankTextGraphic));

    RF_END();
}

modm::ResumableResult<bool> CapBankIndicator::update()
{
    const int BOTTOM = CAP_CENTER_Y - BOX_HEIGHT / 2;
    float voltage = 0;

    RF_BEGIN(1);

    if (capBank != nullptr)
    {
        if (capBank->getStatus() != aruwsrc::communication::sensors::power::Status::UNKNOWN)
        {
            capBankGraphics.graphicData[0].operation =
                capBankGraphics.graphicData[0].operation == Tx::GRAPHIC_DELETE ? Tx::GRAPHIC_ADD
                                                                               : Tx::GRAPHIC_MODIFY;
            capBankGraphics.graphicData[1].operation =
                capBankGraphics.graphicData[1].operation == Tx::GRAPHIC_DELETE ? Tx::GRAPHIC_ADD
                                                                               : Tx::GRAPHIC_MODIFY;
            capBankTextGraphic.graphicData.operation =
                capBankTextGraphic.graphicData.operation == Tx::GRAPHIC_DELETE ? Tx::GRAPHIC_ADD
                                                                               : Tx::GRAPHIC_MODIFY;

            // Update the voltage bar

            // Bottom of the bar is 9v
            // Top of the bar is 21v
            // 9v - 11v is orange
            // 11v - 15v is yellow
            // 15v - 21v is green
            voltage = capBank->getVoltage();

            if (voltage < 9.1)
            {
                voltage = 9.1;
            }

            RefSerialTransmitter::configLine(
                BOX_WIDTH - 20,
                CAP_CENTER_X,
                ((voltage - 9) / (20.5 - 9.0)) * (BOX_HEIGHT - 20) + BOTTOM + 10,
                CAP_CENTER_X,
                BOTTOM + 10,
                &capBankGraphics.graphicData[1]);

            capBankGraphics.graphicData[1].color = static_cast<uint8_t>(
                voltage < 11.0   ? Tx::GraphicColor::ORANGE
                : voltage < 15.0 ? Tx::GraphicColor::YELLOW
                                 : Tx::GraphicColor::GREEN);

            // Update the background status
            capBankGraphics.graphicData[0].color = static_cast<uint8_t>(
                capBank->getStatus() == aruwsrc::communication::sensors::power::Status::RESET
                    ? Tx::GraphicColor::YELLOW
                : capBank->getStatus() ==
                        aruwsrc::communication::sensors::power::Status::CHARGE_DISCHARGE
                    ? Tx::GraphicColor::WHITE
                : capBank->getStatus() ==
                        aruwsrc::communication::sensors::power::Status::DISCHARGING
                    ? Tx::GraphicColor::CYAN
                : capBank->getStatus() == aruwsrc::communication::sensors::power::Status::FAILURE
                    ? Tx::GraphicColor::PURPLISH_RED
                    : Tx::GraphicColor::ORANGE);

            // Update the text
            capBankTextGraphic.graphicData.endAngle = 5;  // Sets the length of the string
            switch (capBank->getStatus())
            {
                case aruwsrc::communication::sensors::power::Status::UNKNOWN:
                    strncpy(capBankTextGraphic.msg, "UNK ", 5);
                    break;
                case aruwsrc::communication::sensors::power::Status::RESET:
                    strncpy(capBankTextGraphic.msg, "RST ", 5);
                    break;
                case aruwsrc::communication::sensors::power::Status::CHARGE_DISCHARGE:
                    strncpy(capBankTextGraphic.msg, "CAPS", 5);
                    break;
                case aruwsrc::communication::sensors::power::Status::SAFE:
                    strncpy(capBankTextGraphic.msg, "SAFE", 5);
                    break;
                case aruwsrc::communication::sensors::power::Status::DISCHARGING:
                    strncpy(capBankTextGraphic.msg, "DIS ", 5);
                    break;
                case aruwsrc::communication::sensors::power::Status::FAILURE:
                    strncpy(capBankTextGraphic.msg, "FAIL", 5);
                    break;
            }

            // Send data
            RF_CALL(refSerialTransmitter.sendGraphic(&capBankGraphics));
            RF_CALL(refSerialTransmitter.sendGraphic(&capBankTextGraphic));
        }
    }

    RF_END();
}

void CapBankIndicator::initialize()
{
    uint8_t capBankName[3];

    getUnusedGraphicName(capBankName);
    RefSerialTransmitter::configGraphicGenerics(
        &capBankGraphics.graphicData[0],
        capBankName,
        Tx::GRAPHIC_DELETE,
        DEFAULT_GRAPHIC_LAYER,
        Tx::GraphicColor::WHITE);

    getUnusedGraphicName(capBankName);
    RefSerialTransmitter::configGraphicGenerics(
        &capBankGraphics.graphicData[1],
        capBankName,
        Tx::GRAPHIC_DELETE,
        DEFAULT_GRAPHIC_LAYER + 1,
        Tx::GraphicColor::GREEN);

    getUnusedGraphicName(capBankName);
    RefSerialTransmitter::configGraphicGenerics(
        &capBankTextGraphic.graphicData,
        capBankName,
        Tx::GRAPHIC_DELETE,
        DEFAULT_GRAPHIC_LAYER,
        Tx::GraphicColor::WHITE);

    if (capBank != nullptr)
    {
        RefSerialTransmitter::configLine(
            BOX_WIDTH,
            CAP_CENTER_X,
            CAP_CENTER_Y + BOX_HEIGHT / 2,
            CAP_CENTER_X,
            CAP_CENTER_Y - BOX_HEIGHT / 2,
            &capBankGraphics.graphicData[0]);

        RefSerialTransmitter::configLine(
            BOX_WIDTH - 20,
            CAP_CENTER_X,
            CAP_CENTER_Y + BOX_HEIGHT / 2 - 10,
            CAP_CENTER_X,
            CAP_CENTER_Y - BOX_HEIGHT / 2 + 10,
            &capBankGraphics.graphicData[1]);

        RefSerialTransmitter::configCharacterMsg(
            15,
            3,
            CAP_CENTER_X - 15 * 2,
            CAP_CENTER_Y + BOX_HEIGHT / 2 + 15 + 5,
            "",
            &capBankTextGraphic);
    }
}
}  // namespace aruwsrc::control::client_display
