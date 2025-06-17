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

#include "cap_bank_text_indicator.hpp"

#include "tap/communication/serial/ref_serial.hpp"
#include "tap/communication/serial/ref_serial_transmitter.hpp"
#include "tap/drivers.hpp"

using namespace tap::communication::serial;

namespace aruwsrc::control::client_display
{
CapBankTextIndicator::CapBankTextIndicator(
    tap::communication::serial::RefSerialTransmitter &refSerialTransmitter,
    const can::capbank::CapacitorBank *capBank)
    : HudIndicator(refSerialTransmitter),
      capBank(capBank)
{
}

modm::ResumableResult<void> CapBankTextIndicator::sendInitialGraphics()
{
    this->previousState = can::capbank::State::UNKNOWN;
    voltageUpdateTimer.restart(500);

    RF_BEGIN(0);

    // remove initial graphics
    RF_CALL(refSerialTransmitter.sendGraphic(&capBankVoltageLevel));
    RF_CALL(refSerialTransmitter.sendGraphic(&capBankTextGraphic));

    RF_END();
}

modm::ResumableResult<void> CapBankTextIndicator::update()
{
    float voltage_squared = 0;
    can::capbank::State state = can::capbank::UNKNOWN;

    RF_BEGIN(1);

    if (capBank == nullptr || !capBank->isOnline())
    {
        RF_RETURN();
    }

    capBankVoltageLevel.graphicData.operation =
        capBankVoltageLevel.graphicData.operation == Tx::GRAPHIC_DELETE ? Tx::GRAPHIC_ADD
                                                                        : Tx::GRAPHIC_MODIFY;
    capBankTextGraphic.graphicData.operation =
        capBankTextGraphic.graphicData.operation == Tx::GRAPHIC_DELETE ? Tx::GRAPHIC_ADD
                                                                       : Tx::GRAPHIC_MODIFY;

    // Update the voltage
    voltage_squared = pow(capBank->getVoltage(), 2);

    if (voltage_squared < VOLTAGE_SQUARED_MIN)
    {
        voltage_squared = VOLTAGE_SQUARED_MIN;
    }

    voltage =
        (voltage_squared - VOLTAGE_SQUARED_MIN) / (VOLTAGE_SQUARED_MAX - VOLTAGE_SQUARED_MIN) * 100;
    RefSerialTransmitter::configInteger(
        SIZE,
        WIDTH,
        NUMBER_X,
        TEXT_Y,
        voltage,
        &capBankVoltageLevel.graphicData);

    // Update the status
    state = capBank->getState();
    switch (state)
    {
        case can::capbank::State::RESET:
            strncpy(capBankTextGraphic.msg, "RST ", 5);
            break;
        case can::capbank::State::SAFE:
            strncpy(capBankTextGraphic.msg, "SAFE", 5);
            break;
        case can::capbank::State::CHARGE:
            strncpy(capBankTextGraphic.msg, "CHRG", 5);
            break;
        case can::capbank::State::CHARGE_DISCHARGE:
            strncpy(capBankTextGraphic.msg, "CHDS", 5);
            break;
        case can::capbank::State::DISCHARGE:
            strncpy(capBankTextGraphic.msg, "DSCH", 5);
            break;
        case can::capbank::State::BATTERY_OFF:
            strncpy(capBankTextGraphic.msg, "BOFF", 5);
            break;
        case can::capbank::State::DISABLED:
            strncpy(capBankTextGraphic.msg, "OFF ", 5);
            break;
        default:
            strncpy(capBankTextGraphic.msg, "UNK ", 5);
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
    if (voltageUpdateTimer.execute())
    {
        RF_CALL(refSerialTransmitter.sendGraphic(&capBankVoltageLevel));
    }

    RF_END();
}

void CapBankTextIndicator::initialize()
{
    uint8_t capBankName[3];

    getUnusedGraphicName(capBankName);
    RefSerialTransmitter::configGraphicGenerics(
        &capBankVoltageLevel.graphicData,
        capBankName,
        Tx::GRAPHIC_ADD,
        DEFAULT_GRAPHIC_LAYER,
        Tx::GraphicColor::ORANGE);

    getUnusedGraphicName(capBankName);
    RefSerialTransmitter::configGraphicGenerics(
        &capBankTextGraphic.graphicData,
        capBankName,
        Tx::GRAPHIC_ADD,
        DEFAULT_GRAPHIC_LAYER,
        Tx::GraphicColor::ORANGE);

    if (capBank != nullptr)
    {
        RefSerialTransmitter::configInteger(
            SIZE,
            WIDTH,
            NUMBER_X,
            TEXT_Y,
            0,
            &capBankVoltageLevel.graphicData);

        RefSerialTransmitter::configCharacterMsg(
            SIZE,
            WIDTH,
            TEXT_X,
            TEXT_Y,
            "",
            &capBankTextGraphic);
    }
}
}  // namespace aruwsrc::control::client_display
