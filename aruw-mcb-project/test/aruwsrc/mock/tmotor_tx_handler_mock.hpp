/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef TMOTOR_TX_HANDLER_MOCK_HPP_
#define TMOTOR_TX_HANDLER_MOCK_HPP_

#include <gmock/gmock.h>

#include "aruwsrc/motor/tmotor_tx_handler.hpp"

namespace aruwsrc::mock
{
class TMotorTxHandlerMock : public aruwsrc::motor::TMotorTxHandler
{
public:
    TMotorTxHandlerMock(aruwsrc::Drivers* drivers);
    virtual ~TMotorTxHandlerMock();

    MOCK_METHOD(void, addMotorToManager, (aruwsrc::motor::Tmotor_AK809 * motor), (override));
    MOCK_METHOD(void, encodeAndSendCanData, (), (override));
    MOCK_METHOD(
        void,
        removeFromMotorManager,
        (const aruwsrc::motor::Tmotor_AK809& motor),
        (override));
    MOCK_METHOD(
        const aruwsrc::motor::Tmotor_AK809 *,
        getCan1Motor,
        (aruwrc::motor::TMotorId motorId),
        (override));
    MOCK_METHOD(
        const aruwsrc::motor::Tmotor_AK809 *,
        getCan2Motor,
        (aruwrc::motor::TMotorId motorId),
        (override));
}; // class TMotorTxHandlerMock

}  // namespace aruwsrc::mock

#endif  // TMOTOR_TX_HANDLER_MOCK_HPP_
