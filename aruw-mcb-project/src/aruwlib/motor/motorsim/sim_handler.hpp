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

    /**
     * Registers a new MotorSim object for the given motor type
     * that will respond at the given position on the given CAN bus.
     * 
     * Default torque loading (0 N*m) is used for this function.
     */
    static void registerSim(MotorSim::MotorType type, aruwlib::can::CanBus bus, uint8_t port);

    /**
     * Overload of earlier registerSim that also allows a torque loading
     * to be specified for the motor sim.
     */
    static void registerSim(MotorSim::MotorType type, float loading, aruwlib::can::CanBus bus, uint8_t port);

    /**
     * Allows the SimHandler to receive a given CAN message
     * and stream input values to the motor sims.
     * Returns true if data is processed (it always should be).
     */
    static bool getMessage(aruwlib::can::CanBus bus, const modm::can::Message& message);

    /**
     * Fills the given pointer with a new motor sim feedback message.
     * Returns true if successful (it always should be).
     */
    static bool sendMessage(aruwlib::can::CanBus bus, modm::can::Message* message);

    /**
     * Updates all MotorSim objects (position, RPM, time values).
     */
    static void updateSims();

    /** Singleton SimHandler instance */
    static SimHandler simHandle;

private:
    /* Constants */
    static const uint8_t CAN_PORTS = 8;
    static const uint8_t CAN_BUSSES = 2;
    static const uint8_t INDEX_LAST_PORT = CAN_PORTS - 1;

    /* Singleton Class Variables */
    static std::array<MotorSim*, CAN_PORTS*CAN_BUSSES> sims; // TODO: should this be an array or vector? or something else?
    static std::array<uint8_t, 2> nextIndex;
};
}
}
#endif
#endif