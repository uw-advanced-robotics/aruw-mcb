/*
 * Copyright (c) 2021-2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "engineer_cv_communication.hpp"

#include "tap/drivers.hpp"

using namespace tap::communication::serial;
using namespace aruwsrc::serial;
using namespace tap::algorithms::transforms;

EngineerCVCommunication* EngineerCVCommunication::engineerCVCommunicationInstance = nullptr;

EngineerCVCommunication::EngineerCVCommunication(tap::Drivers* drivers)
    : DJISerial(drivers, ENGINEER_CV_RX_UART_PORT),
      receptableToCam(Transform::identity())
{
#ifndef ENV_UNIT_TESTS
    // when testing it is OK to have multiple vision coprocessor instances, so this assertion
    // doesn't hold
    assert(engineerCVCommunicationInstance == nullptr);
#endif
    engineerCVCommunicationInstance = this;
}

EngineerCVCommunication::~EngineerCVCommunication() { engineerCVCommunicationInstance = nullptr; }

void EngineerCVCommunication::messageReceiveCallback(const ReceivedSerialMessage& completeMessage)
{
    memcpy(&(targetPositionMessage), &completeMessage.data, sizeof(TargetPositionMessage));
    receptableToCam = Transform(
        targetPositionMessage.posData.xPos,
        targetPositionMessage.posData.yPos,
        targetPositionMessage.posData.zPos,
        targetPositionMessage.rotData.roll,
        targetPositionMessage.rotData.pitch,
        targetPositionMessage.rotData.yaw);
}

void EngineerCVCommunication::initializeCV()
{
    drivers->uart.init<ENGINEER_CV_RX_UART_PORT, ENGINEER_CV_UART_BAUD_RATE>();
}
