/*
 * Copyright (c) 2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of Taproot.
 *
 * Taproot is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Taproot is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Taproot.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef TAPROOT_REF_SERIAL_TRANSMITTER_MOCK_HPP_
#define TAPROOT_REF_SERIAL_TRANSMITTER_MOCK_HPP_

#include <gmock/gmock.h>

#include "tap/communication/serial/ref_serial_transmitter.hpp"

namespace tap::mock
{
class RefSerialTransmitterMock : public tap::communication::serial::RefSerialTransmitter
{
public:
    RefSerialTransmitterMock(Drivers* drivers);
    ~RefSerialTransmitterMock();

    MOCK_METHOD(
        modm::ResumableResult<void>,
        deleteGraphicLayer,
        (Tx::DeleteGraphicOperation, uint8_t),
        (override));
    MOCK_METHOD3(sendGraphicImpl, modm::ResumableResult<void>(Tx::Graphic1Message*, bool, bool));
    MOCK_METHOD3(sendGraphicImpl, modm::ResumableResult<void>(Tx::Graphic2Message*, bool, bool));
    MOCK_METHOD3(sendGraphicImpl, modm::ResumableResult<void>(Tx::Graphic5Message*, bool, bool));
    MOCK_METHOD3(sendGraphicImpl, modm::ResumableResult<void>(Tx::Graphic7Message*, bool, bool));
    MOCK_METHOD3(
        sendGraphicImpl,
        modm::ResumableResult<void>(Tx::GraphicCharacterMessage*, bool, bool));
    virtual modm::ResumableResult<void> sendGraphic(
        Tx::Graphic1Message* msg,
        bool configMsgHeader = true,
        bool sendMsg = true)
    {
        return sendGraphic(msg, configMsgHeader, sendMsg);
    }
    virtual modm::ResumableResult<void> sendGraphic(
        Tx::Graphic2Message* msg,
        bool configMsgHeader = true,
        bool sendMsg = true)
    {
        return sendGraphic(msg, configMsgHeader, sendMsg);
    }
    virtual modm::ResumableResult<void> sendGraphic(
        Tx::Graphic5Message* msg,
        bool configMsgHeader = true,
        bool sendMsg = true)
    {
        return sendGraphic(msg, configMsgHeader, sendMsg);
    }
    virtual modm::ResumableResult<void> sendGraphic(
        Tx::Graphic7Message* msg,
        bool configMsgHeader = true,
        bool sendMsg = true)
    {
        return sendGraphic(msg, configMsgHeader, sendMsg);
    }
    virtual modm::ResumableResult<void> sendGraphic(
        Tx::GraphicCharacterMessage* msg,
        bool configMsgHeader = true,
        bool sendMsg = true)
    {
        return sendGraphic(msg, configMsgHeader, sendMsg);
    }
    MOCK_METHOD(
        modm::ResumableResult<void>,
        sendRobotToRobotMsg,
        (Tx::RobotToRobotMessage*, uint16_t, RobotId, uint16_t),
        (override));
};
}  // namespace tap::mock

#endif  // TAPROOT_REF_SERIAL_TRANSMITTER_MOCK_HPP_
