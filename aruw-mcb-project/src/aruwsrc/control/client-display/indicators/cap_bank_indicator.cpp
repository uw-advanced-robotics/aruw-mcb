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

namespace aruwsrc::control::client_display::indicators
{
CapBankIndicator::CapBankIndicator(
    tap::communication::serial::RefSerialTransmitter &refSerialTransmitter,
    const communication::can::cap_bank::CapacitorBank *capBank)
    : HudIndicator(refSerialTransmitter),
      capBank(capBank),
      numberIndicator(refSerialTransmitter, &numberGraphic, updateVoltage, (int32_t)0)
{
}

modm::ResumableResult<void> CapBankIndicator::sendInitialGraphics()
{
    this->previousState = communication::can::cap_bank::State::UNKNOWN;
    this->previousColor = Tx::GraphicColor::BLACK;

    RF_BEGIN(0);

    // send initial graphics
    RF_CALL(refSerialTransmitter.sendGraphic(&capBankTextGraphic));
    RF_CALL(numberIndicator.initialize());

    RF_END();
}

modm::ResumableResult<void> CapBankIndicator::update()
{
    float voltage_squared = 0;
    communication::can::cap_bank::State state = communication::can::cap_bank::UNKNOWN;

    RF_BEGIN(1);

    if (capBank != nullptr)
    {
        if (capBank->isOnline())
        {
            capBankTextGraphic.graphicData.operation =
                capBankTextGraphic.graphicData.operation == Tx::GRAPHIC_DELETE ? Tx::GRAPHIC_ADD
                                                                               : Tx::GRAPHIC_MODIFY;

            // Update the voltage bar

            voltage_squared = pow(capBank->getVoltage(), 2);

            if (voltage_squared < VOLTAGE_SQUARED_MIN)
            {
                voltage_squared = VOLTAGE_SQUARED_MIN;
            }

            numberGraphic.graphicData.color = static_cast<uint8_t>(
                voltage_squared < VOLTAGE_SQUARED_ORANGE
                    ? Tx::GraphicColor::ORANGE
                    : voltage_squared < VOLTAGE_SQUARED_YELLOW ? Tx::GraphicColor::YELLOW
                                                               : Tx::GraphicColor::GREEN);

            // Update the background status
            state = capBank->getState();
            switch (state)
            {
                case communication::can::cap_bank::State::RESET:
                    strncpy(capBankTextGraphic.msg + 4, "RST ", 5);
                    capBankTextGraphic.graphicData.color =
                        static_cast<uint8_t>(Tx::GraphicColor::YELLOW);
                    break;
                case communication::can::cap_bank::State::SAFE:
                    strncpy(capBankTextGraphic.msg + 4, "SAFE", 5);
                    capBankTextGraphic.graphicData.color =
                        static_cast<uint8_t>(Tx::GraphicColor::ORANGE);
                    break;
                case communication::can::cap_bank::State::CHARGE:
                    strncpy(capBankTextGraphic.msg + 4, "CHRG", 5);
                    capBankTextGraphic.graphicData.color = static_cast<uint8_t>(Tx::GraphicColor::WHITE);
                    break;
                case communication::can::cap_bank::State::CHARGE_DISCHARGE:
                    strncpy(capBankTextGraphic.msg + 4, "CHDS", 5);
                    capBankTextGraphic.graphicData.color = static_cast<uint8_t>(Tx::GraphicColor::WHITE);
                    break;
                case communication::can::cap_bank::State::DISCHARGE:
                    strncpy(capBankTextGraphic.msg + 4, "DSCH", 5);
                    capBankTextGraphic.graphicData.color = static_cast<uint8_t>(Tx::GraphicColor::WHITE);
                    break;
                case communication::can::cap_bank::State::BATTERY_OFF:
                    strncpy(capBankTextGraphic.msg + 4, "BOFF", 5);
                    capBankTextGraphic.graphicData.color = static_cast<uint8_t>(Tx::GraphicColor::CYAN);
                    break;
                case communication::can::cap_bank::State::DISABLED:
                    strncpy(capBankTextGraphic.msg + 4, "OFFL", 5);
                    capBankTextGraphic.graphicData.color =
                        static_cast<uint8_t>(Tx::GraphicColor::PURPLISH_RED);
                    break;
                default:
                    strncpy(capBankTextGraphic.msg + 4, "UNKN", 5);
                    capBankTextGraphic.graphicData.color =
                        static_cast<uint8_t>(Tx::GraphicColor::YELLOW);
                    break;
            }
            // Update the text
            capBankTextGraphic.graphicData.endAngle = 9;  // Sets the length of the string

            if (numberGraphic.graphicData.color != static_cast<uint8_t>(this->previousColor))
            {
                this->previousColor =
                    static_cast<Tx::GraphicColor>(numberGraphic.graphicData.color);
            }

            numberIndicator.setIndicatorState(voltage_squared / VOLTAGE_SQUARED_MAX);
            RF_CALL(numberIndicator.draw());
        }

        // Send data
        if (state != this->previousState)
        {
            this->previousState = state;
            RF_CALL(refSerialTransmitter.sendGraphic(&capBankTextGraphic));
        }
    }

    RF_END();
}

void CapBankIndicator::initialize()
{
    uint8_t graphicName[3];

    getUnusedGraphicName(graphicName);
    RefSerialTransmitter::configGraphicGenerics(
        &capBankTextGraphic.graphicData,
        graphicName,
        Tx::GRAPHIC_ADD,
        DEFAULT_GRAPHIC_LAYER,
        Tx::GraphicColor::WHITE);

    getUnusedGraphicName(graphicName);
    RefSerialTransmitter::configGraphicGenerics(
        &numberGraphic.graphicData,
        graphicName,
        Tx::GRAPHIC_ADD,
        DEFAULT_GRAPHIC_LAYER,
        Tx::GraphicColor::WHITE);

    if (capBank != nullptr)
    {
        RefSerialTransmitter::configCharacterMsg(
            SIZE,
            WIDTH,
            TEXT_X,
            TEXT_Y,
            "CAP:",
            &capBankTextGraphic);

        strncpy(capBankTextGraphic.msg + 4, "UNKN", 5);
        capBankTextGraphic.graphicData.color = static_cast<uint8_t>(Tx::GraphicColor::YELLOW);
        capBankTextGraphic.graphicData.endAngle = 9;

        updateVoltage(0, &numberGraphic);
    }
}
}  // namespace aruwsrc::control::client_display::indicators
