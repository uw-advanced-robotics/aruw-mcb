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
    tap::communication::serial::RefSerialTransmitter &refSerialTransmitter,
    const can::capbank::CapacitorBank *capBank)
    : HudIndicator(refSerialTransmitter),
      capBank(capBank)
{
}

modm::ResumableResult<void> CapBankIndicator::sendInitialGraphics()
{
    this->previousState = can::capbank::State::UNKNOWN;
    this->previousColor = Tx::GraphicColor::BLACK;
    voltageUpdateTimer.restart(500);

    RF_BEGIN(0);

    // remove initial graphics
    RF_CALL(refSerialTransmitter.sendGraphic(&capBankBackgroundLine));
    RF_CALL(refSerialTransmitter.sendGraphic(&capBankVoltageLevel));
    RF_CALL(refSerialTransmitter.sendGraphic(&capBankTextGraphic));

    RF_END();
}

modm::ResumableResult<void> CapBankIndicator::update()
{
    const int BOTTOM = CAP_CENTER_Y - BOX_HEIGHT / 2;
    float voltage_squared = 0;
    can::capbank::State state = can::capbank::UNKNOWN;

    RF_BEGIN(1);

    if (capBank != nullptr)
    {
        if (capBank->isOnline())
        {
            capBankBackgroundLine.graphicData.operation =
                capBankBackgroundLine.graphicData.operation == Tx::GRAPHIC_DELETE
                    ? Tx::GRAPHIC_ADD
                    : Tx::GRAPHIC_MODIFY;
            capBankVoltageLevel.graphicData.operation =
                capBankVoltageLevel.graphicData.operation == Tx::GRAPHIC_DELETE
                    ? Tx::GRAPHIC_ADD
                    : Tx::GRAPHIC_MODIFY;
            capBankTextGraphic.graphicData.operation =
                capBankTextGraphic.graphicData.operation == Tx::GRAPHIC_DELETE ? Tx::GRAPHIC_ADD
                                                                               : Tx::GRAPHIC_MODIFY;

            // Update the voltage bar

            voltage_squared = pow(capBank->getVoltage(), 2);

            if (voltage_squared < VOLTAGE_SQUARED_MIN)
            {
                voltage_squared = VOLTAGE_SQUARED_MIN;
            }

            RefSerialTransmitter::configLine(
                BOX_WIDTH - 20,
                CAP_CENTER_X,
                std::min(
                    (voltage_squared - VOLTAGE_SQUARED_MIN) /
                        (VOLTAGE_SQUARED_MAX - VOLTAGE_SQUARED_MIN),
                    1.0f) *
                        (BOX_HEIGHT - 20) +
                    BOTTOM + 10,
                CAP_CENTER_X,
                BOTTOM + 10,
                &capBankVoltageLevel.graphicData);

            capBankVoltageLevel.graphicData.color = static_cast<uint8_t>(
                voltage_squared < VOLTAGE_SQUARED_ORANGE
                    ? Tx::GraphicColor::ORANGE
                    : voltage_squared < VOLTAGE_SQUARED_YELLOW ? Tx::GraphicColor::YELLOW
                                                               : Tx::GraphicColor::GREEN);

            // Update the background status
            state = capBank->getState();
            switch (state)
            {
                case can::capbank::State::RESET:
                    strncpy(capBankTextGraphic.msg, "RST ", 5);
                    capBankBackgroundLine.graphicData.color =
                        static_cast<uint8_t>(Tx::GraphicColor::YELLOW);
                    break;
                case can::capbank::State::SAFE:
                    strncpy(capBankTextGraphic.msg, "SAFE", 5);
                    capBankBackgroundLine.graphicData.color =
                        static_cast<uint8_t>(Tx::GraphicColor::ORANGE);
                    break;
                case can::capbank::State::CHARGE:
                    strncpy(capBankTextGraphic.msg, "CHRG", 5);
                    capBankBackgroundLine.graphicData.color =
                        static_cast<uint8_t>(Tx::GraphicColor::WHITE);
                    break;
                case can::capbank::State::CHARGE_DISCHARGE:
                    strncpy(capBankTextGraphic.msg, "CHDS", 5);
                    capBankBackgroundLine.graphicData.color =
                        static_cast<uint8_t>(Tx::GraphicColor::WHITE);
                    break;
                case can::capbank::State::DISCHARGE:
                    strncpy(capBankTextGraphic.msg, "DSCH", 5);
                    capBankBackgroundLine.graphicData.color =
                        static_cast<uint8_t>(Tx::GraphicColor::WHITE);
                    break;
                case can::capbank::State::BATTERY_OFF:
                    strncpy(capBankTextGraphic.msg, "BOFF", 5);
                    capBankBackgroundLine.graphicData.color =
                        static_cast<uint8_t>(Tx::GraphicColor::CYAN);
                    break;
                case can::capbank::State::DISABLED:
                    strncpy(capBankTextGraphic.msg, "OFF ", 5);
                    capBankBackgroundLine.graphicData.color =
                        static_cast<uint8_t>(Tx::GraphicColor::PURPLISH_RED);
                    break;
                default:
                    strncpy(capBankTextGraphic.msg, "UNK ", 5);
                    capBankBackgroundLine.graphicData.color =
                        static_cast<uint8_t>(Tx::GraphicColor::YELLOW);
                    break;
            }
            // Update the text
            capBankTextGraphic.graphicData.endAngle = 5;  // Sets the length of the string

            // Send data
            if (state != this->previousState)
            {
                this->previousState = state;
                RF_CALL(refSerialTransmitter.sendGraphic(&capBankTextGraphic));
            }
            if (capBankBackgroundLine.graphicData.color !=
                static_cast<uint8_t>(this->previousColor))
            {
                this->previousColor =
                    static_cast<Tx::GraphicColor>(capBankBackgroundLine.graphicData.color);
                RF_CALL(refSerialTransmitter.sendGraphic(&capBankBackgroundLine));
            }
            if (voltageUpdateTimer.execute())
            {
                RF_CALL(refSerialTransmitter.sendGraphic(&capBankVoltageLevel));
            }
        }
    }

    RF_END();
}

void CapBankIndicator::initialize()
{
    uint8_t capBankName[3];

    getUnusedGraphicName(capBankName);
    RefSerialTransmitter::configGraphicGenerics(
        &capBankBackgroundLine.graphicData,
        capBankName,
        Tx::GRAPHIC_DELETE,
        DEFAULT_GRAPHIC_LAYER,
        Tx::GraphicColor::WHITE);

    getUnusedGraphicName(capBankName);
    RefSerialTransmitter::configGraphicGenerics(
        &capBankVoltageLevel.graphicData,
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
            &capBankBackgroundLine.graphicData);

        RefSerialTransmitter::configLine(
            BOX_WIDTH - 20,
            CAP_CENTER_X,
            CAP_CENTER_Y + BOX_HEIGHT / 2 - 10,
            CAP_CENTER_X,
            CAP_CENTER_Y - BOX_HEIGHT / 2 + 10,
            &capBankVoltageLevel.graphicData);

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
