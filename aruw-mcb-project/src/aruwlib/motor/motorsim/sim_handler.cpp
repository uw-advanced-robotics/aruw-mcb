/*
 * Copyright (c) 2020 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruw-mcb.
 *
 * aruw-mcb is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any latersd version.
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

#include "aruwlib/motor/dji_motor.hpp"

#include "can_serializer.hpp"
#include "motor_sim.hpp"

namespace aruwlib
{
namespace motorsim
{
SimHandler::SimHandler() { resetMotorSims(); }

void SimHandler::resetMotorSims()
{
    // for-loop used in case addiitonal Can busses are utilized
    for (int i = 0; i < nextIndex.size(); i++)
    {
        nextIndex[i] = 0;
    }

    for (int i = 0; i < sims.size(); i++)
    {
        if (sims[i] != nullptr)
        {
            sims[i]->reset();
        }
    }
}

void SimHandler::registerSim(
    MotorSim::MotorType type,
    aruwlib::can::CanBus bus,
    aruwlib::motor::MotorId id)
{
    int8_t port = CanSerializer::idToPort(id);
    if (port < CAN_PORTS)
    {
        switch (bus)
        {
            case aruwlib::can::CanBus::CAN_BUS1:
                if (sims[port] == nullptr)
                {
                    sims[port] = &MotorSim(type);
                }
                break;

            case aruwlib::can::CanBus::CAN_BUS2:
                if (sims[port + INDEX_LAST_PORT] == nullptr)
                {
                    sims[port + INDEX_LAST_PORT] = &MotorSim(type);
                }
                break;
        }
    }
}

void SimHandler::registerSim(
    MotorSim::MotorType type,
    float loading,
    aruwlib::can::CanBus bus,
    aruwlib::motor::MotorId id)
{
    int8_t port = CanSerializer::idToPort(id);
    if (port < CAN_PORTS)
    {
        switch (bus)
        {
            case aruwlib::can::CanBus::CAN_BUS1:
                if (sims[port] == nullptr)
                {
                    sims[port] = &MotorSim(type, loading);
                }
                break;

            case aruwlib::can::CanBus::CAN_BUS2:
                if (sims[port + INDEX_LAST_PORT] == nullptr)
                {
                    sims[port + INDEX_LAST_PORT] = &MotorSim(type, loading);
                }
                break;
        }
    }
}

bool SimHandler::getMessage(aruwlib::can::CanBus bus, const modm::can::Message& message)
{
    modm::can::Message m = message;
    std::array<int16_t, 4> newInputs = CanSerializer::parseMessage(&m);

    int startingIndex;
    switch (bus)
    {
        case aruwlib::can::CanBus::CAN_BUS1:
            if (m.getIdentifier() == 0x1FF)
            {
                startingIndex = 0;
            }
            else if (m.getIdentifier() == 0x200)
            {
                startingIndex = CAN_PORTS / 2;
            }
            break;

        case aruwlib::can::CanBus::CAN_BUS2:
            if (m.getIdentifier() == 0x1FF)
            {
                startingIndex = INDEX_LAST_PORT + 1;
            }
            else if (m.getIdentifier() == 0x200)
            {
                startingIndex = (INDEX_LAST_PORT + 1) + CAN_PORTS / 2;
            }
            break;

        default:
            return false;
    }

    for (int i = 0; i < 4; i++)
    {
        if (sims[startingIndex + i] != nullptr)
        {
            sims[startingIndex + i]->setInput(newInputs[i]);
        }
    }

    return true;
}

bool SimHandler::sendMessage(aruwlib::can::CanBus bus, modm::can::Message* message)
{
    uint8_t busInt;

    switch (bus)
    {
        case aruwlib::can::CanBus::CAN_BUS1:
            busInt = 0;
            break;

        case aruwlib::can::CanBus::CAN_BUS2:
            busInt = 1;
            break;

        default:
            return false;
    }
    // TODO: Is this proper C++ syntax?
    message = CanSerializer::serializeFeedback(
        sims[nextIndex[busInt]]->getEnc(),
        sims[nextIndex[busInt]]->getRPM(),
        sims[nextIndex[busInt]]->getInput(),
        nextIndex[busInt]);

    nextIndex[busInt]++;
    if (nextIndex[busInt] > INDEX_LAST_PORT)
    {
        nextIndex[busInt] = 0;
    }

    return true;
}

void SimHandler::updateSims()
{
    for (int i = 0; i < sims.size(); i++)
    {
        if (sims[i] != nullptr)
        {
            sims[i]->update();
        }
    }
}

}  // namespace motorsim
}  // namespace aruwlib
#endif