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
#ifndef sim_handler_hpp_

#define sim_handler_hpp_

#include <vector>
#include <array>
#include "motor_sim.hpp"
#include "aruwlib/communication/can/can.hpp"
#include "modm/platform.hpp"

namespace aruwlib
{
namespace motorsim
{
class SimHandler
{
public:
    SimHandler();

    static void registerSim(MotorSim::MotorType type, uint8_t port);

    static void registerSim(MotorSim::MotorType type, float loading, uint8_t port);

    static bool getMessage(aruwlib::can::CanBus bus, const modm::can::Message& message);

    static bool sendMessage(modm::can::Message* message);

    static void updateSims();

    static SimHandler simHandle;

private:
    static const uint8_t capacity = 16;
    static std::array<MotorSim*, capacity> sims; // TODO: should this be an array or vector? or something else?
    static uint8_t nextSendIndex;
};
}
}
#endif
#endif