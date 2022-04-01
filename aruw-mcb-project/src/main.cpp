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
#include "aruwsrc/drivers_singleton.hpp"

/* error handling includes --------------------------------------------------*/
#include "tap/errors/create_errors.hpp"

/* control includes ---------------------------------------------------------*/
#include "tap/architecture/clock.hpp"

#include "aruwsrc/control/robot_control.hpp"
#include "aruwsrc/sim-initialization/robot_sim.hpp"
#include "aruwsrc/util_macros.hpp"
#include "tap/communication/gpio/pwm.hpp"
#include "tap/communication/gpio/digital.hpp"

/* define timers here -------------------------------------------------------*/
tap::arch::PeriodicMilliTimer sendMotorTimeout(2);
tap::arch::PeriodicMilliTimer sendVisionCoprocessorTimeout(3);
tap::gpio::Pwm::Pin pulse = tap::gpio::Pwm::Pin::X;
tap::gpio::Digital::OutputPin direction = tap::gpio::Digital::OutputPin::E;

// Place any sort of input/output initialization here. For example, place
// serial init stuff here.
static void initializeIo(aruwsrc::Drivers *drivers);

// Anything that you would like to be called place here. It will be called
// very frequently. Use PeriodicMilliTimers if you don't want something to be
// called as frequently.
static void updateIo(aruwsrc::Drivers *drivers);
using namespace tap::gpio;
int main()
{
#ifdef PLATFORM_HOSTED
    std::cout << "Simulation starting..." << std::endl;
#endif

    /*
     * NOTE: We are using DoNotUse_getDrivers here because in the main
     *      robot loop we must access the singleton drivers to update
     *      IO states and run the scheduler.
     */
    aruwsrc::Drivers *drivers = aruwsrc::DoNotUse_getDrivers();

    Board::initialize();
    initializeIo(drivers);
    drivers->pwm.setTimerFrequency(tap::gpio::Pwm::Timer::TIMER8, 100);


#ifdef PLATFORM_HOSTED
    aruwsrc::sim::initialize_robot_sim();
    tap::motorsim::SimHandler::resetMotorSims();
    // Blocking call, waits until Windows Simulator connects.
    tap::communication::TCPServer::MainServer()->getConnection();
#endif
    drivers->digital.set(direction, true);
    while (1)
    {
        drivers->pwm.write(0.5, pulse);
    }
    return 0;
}

static void initializeIo(aruwsrc::Drivers *drivers)
{
    drivers->analog.init();
    drivers->pwm.init();
    drivers->digital.init();
    drivers->leds.init();
    drivers->can.initialize();
    drivers->errorController.init();
    drivers->remote.initialize();
    drivers->mpu6500.init();
    drivers->refSerial.initialize();
    drivers->terminalSerial.initialize();
    drivers->oledDisplay.initialize();
    drivers->schedulerTerminalHandler.init();
    drivers->djiMotorTerminalSerialHandler.init();
    drivers->legacyVisionCoprocessor.initializeCV();
    drivers->mpu6500TerminalSerialHandler.init();
#if defined(ALL_SOLDIERS) || defined(TARGET_HERO)
    drivers->turretMCBCanComm.init();
#endif
}

static void updateIo(aruwsrc::Drivers *drivers)
{
#ifdef PLATFORM_HOSTED
    tap::motorsim::SimHandler::updateSims();
#endif

    drivers->canRxHandler.pollCanData();
    drivers->refSerial.updateSerial();
    drivers->remote.read();
    drivers->oledDisplay.updateDisplay();
    drivers->mpu6500.read();
    drivers->legacyVisionCoprocessor.updateSerial();
}
