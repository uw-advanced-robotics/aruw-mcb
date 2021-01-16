/*
 * Copyright (c) 2020 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifdef PLATFORM_HOSTED

#include "sim_handler.hpp"
#include <iostream>

namespace aruwlib
{
namespace motorsim
{
/* Singleton Class Variables */
std::array<
    std::array<MotorSim*, SimHandler::CAN_BUSSES>,
    aruwlib::motor::DjiMotorTxHandler::DJI_MOTORS_PER_CAN>
    SimHandler::sims;
std::array<uint8_t, SimHandler::CAN_BUSSES> SimHandler::nextCanSendIndex;

SimHandler::SimHandler() { resetMotorSims(); }

void SimHandler::resetMotorSims()
{
    // for-loop used in case addiitonal Can busses are utilized
    for (uint32_t i = 0; i < nextCanSendIndex.size(); i++)
    {
        nextCanSendIndex[i] = 0;
    }

    for (uint32_t i = 0; i < sims.size(); i++)
    {
        for (uint32_t j = 0; j < sims[i].size(); j++) 
        {
            if (sims[i][j] != nullptr)
            {
                sims[i][j]->reset();
            }
        }
    }
}

void SimHandler::registerSim(
    MotorSim::MotorType type,
    aruwlib::can::CanBus bus,
    aruwlib::motor::MotorId id,
    float loading)
{
    int8_t port = CanSerializer::idToPort(id);
    int8_t busIndex = static_cast<uint8_t>(bus);

    if (sims[busIndex][port] == nullptr)
    {
        // TODO: Does this correctly statically allocate the pointer at sims[busIndex][port]?
        sims[busIndex][port] = new motorsim::MotorSim(type, loading);
    }
}

bool SimHandler::getMessage(aruwlib::can::CanBus bus, const modm::can::Message& message)
{
    std::array<int16_t, 4> newInputs = CanSerializer::parseMessage(&message);
    uint8_t busIndex = static_cast<uint8_t>(bus);

    if (message.getIdentifier() == 0x1FF)
    {
        for (int i = 0; i < (aruwlib::motor::DjiMotorTxHandler::DJI_MOTORS_PER_CAN / 2); i++)
        {
            if (sims[busIndex][i] != nullptr)
            {
                sims[busIndex][i]->setInput(newInputs[i]);
            }
        }
    }

    else if (message.getIdentifier() == 0x200)
    {
        for (int i = (aruwlib::motor::DjiMotorTxHandler::DJI_MOTORS_PER_CAN / 2);
             i < aruwlib::motor::DjiMotorTxHandler::DJI_MOTORS_PER_CAN;
             i++)
        {
            if (sims[busIndex][i] != nullptr)
            {
                sims[busIndex][i]->setInput(newInputs[i]);
            }
        }
    }

    else
        return false;

    return true;
}

bool SimHandler::sendMessage(aruwlib::can::CanBus bus, modm::can::Message* message)
{
    uint8_t busIndex = static_cast<uint8_t>(bus);

    // Check to make sure sim actually exists.
    if (sims[busIndex][nextCanSendIndex[busIndex]] == nullptr) {
        return false;
    }
    *message = CanSerializer::serializeFeedback(
        sims[busIndex][nextCanSendIndex[busIndex]]->getEnc(),
        sims[busIndex][nextCanSendIndex[busIndex]]->getRPM(),
        sims[busIndex][nextCanSendIndex[busIndex]]->getInput(),
        nextCanSendIndex[busIndex]);

    nextCanSendIndex[busIndex]++;
    if (nextCanSendIndex[busIndex] > INDEX_LAST_PORT)
    {
        nextCanSendIndex[busIndex] = 0;
    }
    return true;
}

void SimHandler::updateSims()
{
    for (uint32_t i = 0; i < sims.size(); i++)
    {
        for (uint32_t j = 0; j < sims[0].size(); j++)
        {
            if (sims[i][j] != nullptr)
            {
                sims[i][j]->update();
            }
        }
    }
}

}  // namespace motorsim

}  // namespace aruwlib

#endif  // PLATFORM_HOSTED