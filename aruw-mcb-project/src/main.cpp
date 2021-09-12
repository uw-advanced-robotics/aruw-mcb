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

#include "tap/communication/tcp-server/tcp_server.hpp"
#include "tap/motor/motorsim/sim_handler.hpp"
#endif

#include "tap/board/board.hpp"

#include "modm/architecture/interface/delay.hpp"

/* arch includes ------------------------------------------------------------*/
#include "tap/architecture/periodic_timer.hpp"
#include "tap/architecture/profiler.hpp"

/* communication includes ---------------------------------------------------*/
#include "tap/drivers_singleton.hpp"

/* error handling includes --------------------------------------------------*/
#include "tap/errors/create_errors.hpp"

/* control includes ---------------------------------------------------------*/
#include "tap/architecture/clock.hpp"

#include "aruwlib/architecture/endianness_wrappers.hpp"

#include "aruwsrc/control/robot_control.hpp"
#include "modm/architecture/interface/can_message.hpp"

using tap::Drivers;

/* define timers here -------------------------------------------------------*/
tap::arch::PeriodicMilliTimer sendMotorTimeout(2);
tap::arch::PeriodicMilliTimer sendXavierTimeout(3);

// Place any sort of input/output initialization here. For example, place
// serial init stuff here.
static void initializeIo(tap::Drivers *drivers);

// Anything that you would like to be called place here. It will be called
// very frequently. Use PeriodicMilliTimers if you don't want something to be
// called as frequently.
static void updateIo(tap::Drivers *drivers);

static constexpr uint16_t IMU_MSG_CAN_ID = 0x203;

int main()
{
    /*
     * NOTE: We are using DoNotUse_getDrivers here because in the main
     *      robot loop we must access the singleton drivers to update
     *      IO states and run the scheduler.
     */
    tap::Drivers *drivers = tap::DoNotUse_getDrivers();

    Board::initialize();
    initializeIo(drivers);

    int i = 0;

    while (1)
    {
        // do this as fast as you can
        PROFILE(drivers->profiler, updateIo, (drivers));

        if (sendXavierTimeout.execute())
        {
            PROFILE(drivers->profiler, drivers->xavierSerial.sendMessage, ());
            // TODO try faster baude rate so we can send more frequently (currently mcb's serial
            // buffers are overflowing if you try and send faster than 3 ms).
        }

        if (sendMotorTimeout.execute())
        {
            PROFILE(drivers->profiler, drivers->mpu6500.periodicIMUUpdate, ());
            PROFILE(drivers->profiler, drivers->terminalSerial.update, ());

            float yaw = drivers->mpu6500.getYaw();
            int16_t gzRaw = drivers->mpu6500.getGz() * tap::sensors::Mpu6500::LSB_D_PER_S_TO_D_PER_S;

            if (drivers->can.isReadyToSend(aruwlib::can::CanBus::CAN_BUS1))
            {

                drivers->leds.set(aruwlib::gpio::Leds::Green, i < 50);
                i = (i + 1) % 100;

                modm::can::Message msg(IMU_MSG_CAN_ID, 8);
                msg.setExtended(false);
                aruwlib::arch::convertToLittleEndian(yaw, msg.data);
                aruwlib::arch::convertToLittleEndian(gzRaw, msg.data + 4);
                drivers->can.sendMessage(aruwlib::can::CanBus::CAN_BUS1, msg);
            }
        }
        modm::delay_us(10);
    }
    return 0;
}

static void initializeIo(tap::Drivers *drivers)
{
    drivers->can.initialize();
    drivers->leds.init();
    drivers->digital.init();
    drivers->mpu6500.init();
    drivers->terminalSerial.initialize();
    drivers->errorController.init();
    drivers->mpu6500TerminalSerialHandler.init();
}

static void updateIo(tap::Drivers *drivers)
{
    drivers->mpu6500.read();
    drivers->canRxHandler.pollCanData();
}
