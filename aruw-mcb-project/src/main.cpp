/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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
/* hosted environment (simulator) includes --------------------------------- */
#include <iostream>

#include <aruwlib/communication/tcp-server/TCPServer.hpp>
#include <aruwlib/motor/motorsim/sim_handler.hpp>
#endif

#include <aruwlib/rm-dev-board-a/board.hpp>
#include <modm/architecture/interface/delay.hpp>

/* arch includes ------------------------------------------------------------*/
#include <aruwlib/architecture/periodic_timer.hpp>
#include <aruwlib/architecture/profiler.hpp>

/* communication includes ---------------------------------------------------*/
#include <aruwlib/DriversSingleton.hpp>

/* error handling includes --------------------------------------------------*/
#include <aruwlib/errors/create_errors.hpp>

/* control includes ---------------------------------------------------------*/
#include <aruwlib/architecture/clock.hpp>

#include "aruwsrc/control/robot_control.hpp"

#include "aruwlib/architecture/endianness_wrappers.hpp"

using aruwlib::Drivers;

/* define timers here -------------------------------------------------------*/
aruwlib::arch::PeriodicMilliTimer sendMotorTimeout(2);

// Place any sort of input/output initialization here. For example, place
// serial init stuff here.
static void initializeIo(aruwlib::Drivers *drivers);

// Anything that you would like to be called place here. It will be called
// very frequently. Use PeriodicMilliTimers if you don't want something to be
// called as frequently.
static void updateIo(aruwlib::Drivers *drivers);

int main()
{
    /*
     * NOTE: We are using DoNotUse_getDrivers here because in the main
     *      robot loop we must access the singleton drivers to update
     *      IO states and run the scheduler.
     */
    aruwlib::Drivers *drivers = aruwlib::DoNotUse_getDrivers();

    Board::initialize();
    initializeIo(drivers);

    while (1)
    {
        // do this as fast as you can
        PROFILE(drivers->profiler, updateIo, (drivers));

        if (sendMotorTimeout.execute())
        {
            PROFILE(drivers->profiler, drivers->mpu6500.calcIMUAngles, ());
            PROFILE(drivers->profiler, drivers->errorController.updateLedDisplay, ());

            // I think these are all between -180 and 180 but I'm not sure
            float pitch = drivers->mpu6500.getPitch();
            float yaw = drivers->mpu6500.getYaw();
            // TODO check how large these get
            int16_t gxRaw = drivers->mpu6500.getGx();
            int16_t gzRaw = drivers->mpu6500.getGz();

            if (drivers->can.isReadyToSend(aruwlib::can::CanBus::CAN_BUS1))
            {
                // We need to send > 8 bytes, so send two messages
                modm::can::Message msg(0x201, 8);
                msg.setExtended(false);
                aruwlib::arch::convertToLittleEndian(static_cast<int16_t>(pitch * 100.0f), msg.data);
                aruwlib::arch::convertToLittleEndian(static_cast<int16_t>(yaw * 100.0f), msg.data + 2);
                aruwlib::arch::convertToLittleEndian(gxRaw, msg.data + 4);
                aruwlib::arch::convertToLittleEndian(gzRaw, msg.data + 6);
                drivers->can.sendMessage(aruwlib::can::CanBus::CAN_BUS1, msg);
            }
        }
        modm::delay_us(10);
    }
    return 0;
}

static void initializeIo(aruwlib::Drivers *drivers)
{
    drivers->leds.init();
    drivers->can.initialize();
    drivers->errorController.init();
    drivers->mpu6500.init();
}

static void updateIo(aruwlib::Drivers *drivers)
{
    drivers->mpu6500.read();
}
