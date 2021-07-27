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

#include "aruwlib/communication/tcp-server/tcp_server.hpp"
#include "aruwlib/motor/motorsim/sim_handler.hpp"
#endif

#include "aruwlib/rm-dev-board-a/board.hpp"

#include "modm/architecture/interface/delay.hpp"

/* arch includes ------------------------------------------------------------*/
#include "aruwlib/architecture/periodic_timer.hpp"
#include "aruwlib/architecture/profiler.hpp"

/* communication includes ---------------------------------------------------*/
#include "aruwlib/drivers_singleton.hpp"

/* error handling includes --------------------------------------------------*/
#include "aruwlib/errors/create_errors.hpp"

/* control includes ---------------------------------------------------------*/
#include "aruwlib/architecture/clock.hpp"

#include "aruwsrc/control/robot_control.hpp"
#include "aruwsrc/sim-initialization/robot_sim.hpp"

using aruwlib::Drivers;

/* define timers here -------------------------------------------------------*/
aruwlib::arch::PeriodicMilliTimer sendMotorTimeout(2);
aruwlib::arch::PeriodicMilliTimer sendXavierTimeout(3);

// Place any sort of input/output initialization here. For example, place
// serial init stuff here.
static void initializeIo(aruwlib::Drivers *drivers);

// Anything that you would like to be called place here. It will be called
// very frequently. Use PeriodicMilliTimers if you don't want something to be
// called as frequently.
static void updateIo(aruwlib::Drivers *drivers);


void playMarioThemesongBlocking(gpio::Pwm *pwmController)
{
    playNote(pwmController, 660);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(150);
    playNote(pwmController, 660);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(300);
    playNote(pwmController, 660);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(300);
    playNote(pwmController, 510);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(100);
    playNote(pwmController, 660);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(300);
    playNote(pwmController, 770);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(550);
    playNote(pwmController, 380);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(575);
    playNote(pwmController, 510);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(450);
    playNote(pwmController, 380);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(400);
    playNote(pwmController, 320);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(500);
    playNote(pwmController, 440);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(300);
    playNote(pwmController, 480);
    modm::delay_ms(80);
    silenceBuzzer(pwmController);
    modm::delay_ms(330);
    playNote(pwmController, 450);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(150);
    playNote(pwmController, 430);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(300);
    playNote(pwmController, 380);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(200);
    playNote(pwmController, 660);
    modm::delay_ms(80);
    silenceBuzzer(pwmController);
    modm::delay_ms(200);
    playNote(pwmController, 760);
    modm::delay_ms(50);
    silenceBuzzer(pwmController);
    modm::delay_ms(150);
    playNote(pwmController, 860);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(300);
    playNote(pwmController, 700);
    modm::delay_ms(80);
    silenceBuzzer(pwmController);
    modm::delay_ms(150);
    playNote(pwmController, 760);
    modm::delay_ms(50);
    silenceBuzzer(pwmController);
    modm::delay_ms(350);
    playNote(pwmController, 660);
    modm::delay_ms(80);
    silenceBuzzer(pwmController);
    modm::delay_ms(300);
    playNote(pwmController, 520);
    modm::delay_ms(80);
    silenceBuzzer(pwmController);
    modm::delay_ms(150);
    playNote(pwmController, 580);
    modm::delay_ms(80);
    silenceBuzzer(pwmController);
    modm::delay_ms(150);
    playNote(pwmController, 480);
    modm::delay_ms(80);
    silenceBuzzer(pwmController);
    modm::delay_ms(500);
    playNote(pwmController, 510);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(450);
    playNote(pwmController, 380);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(400);
    playNote(pwmController, 320);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(500);
    playNote(pwmController, 440);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(300);
    playNote(pwmController, 480);
    modm::delay_ms(80);
    silenceBuzzer(pwmController);
    modm::delay_ms(330);
    playNote(pwmController, 450);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(150);
    playNote(pwmController, 430);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(300);
    playNote(pwmController, 380);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(200);
    playNote(pwmController, 660);
    modm::delay_ms(80);
    silenceBuzzer(pwmController);
    modm::delay_ms(200);
    playNote(pwmController, 760);
    modm::delay_ms(50);
    silenceBuzzer(pwmController);
    modm::delay_ms(150);
    playNote(pwmController, 860);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(300);
    playNote(pwmController, 700);
    modm::delay_ms(80);
    silenceBuzzer(pwmController);
    modm::delay_ms(150);
    playNote(pwmController, 760);
    modm::delay_ms(50);
    silenceBuzzer(pwmController);
    modm::delay_ms(350);
    playNote(pwmController, 660);
    modm::delay_ms(80);
    silenceBuzzer(pwmController);
    modm::delay_ms(300);
    playNote(pwmController, 520);
    modm::delay_ms(80);
    silenceBuzzer(pwmController);
    modm::delay_ms(150);
    playNote(pwmController, 580);
    modm::delay_ms(80);
    silenceBuzzer(pwmController);
    modm::delay_ms(150);
    playNote(pwmController, 480);
    modm::delay_ms(80);
    silenceBuzzer(pwmController);
    modm::delay_ms(500);
    playNote(pwmController, 500);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(300);
    playNote(pwmController, 760);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(100);
    playNote(pwmController, 720);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(150);
    playNote(pwmController, 680);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(150);
    playNote(pwmController, 620);
    modm::delay_ms(150);
    silenceBuzzer(pwmController);
    modm::delay_ms(300);
    playNote(pwmController, 650);
    modm::delay_ms(150);
    silenceBuzzer(pwmController);
    modm::delay_ms(300);
    playNote(pwmController, 380);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(150);
    playNote(pwmController, 430);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(150);
    playNote(pwmController, 500);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(300);
    playNote(pwmController, 430);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(150);
    playNote(pwmController, 500);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(100);
    playNote(pwmController, 570);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(220);
    playNote(pwmController, 500);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(300);
    playNote(pwmController, 760);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(100);
    playNote(pwmController, 720);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(150);
    playNote(pwmController, 680);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(150);
    playNote(pwmController, 620);
    modm::delay_ms(150);
    silenceBuzzer(pwmController);
    modm::delay_ms(300);
    playNote(pwmController, 650);
    modm::delay_ms(200);
    silenceBuzzer(pwmController);
    modm::delay_ms(300);
    playNote(pwmController, 1020);
    modm::delay_ms(80);
    silenceBuzzer(pwmController);
    modm::delay_ms(300);
    playNote(pwmController, 1020);
    modm::delay_ms(80);
    silenceBuzzer(pwmController);
    modm::delay_ms(150);
    playNote(pwmController, 1020);
    modm::delay_ms(80);
    silenceBuzzer(pwmController);
    modm::delay_ms(300);
    playNote(pwmController, 380);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(300);
    playNote(pwmController, 500);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(300);
    playNote(pwmController, 760);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(100);
    playNote(pwmController, 720);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(150);
    playNote(pwmController, 680);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(150);
    playNote(pwmController, 620);
    modm::delay_ms(150);
    silenceBuzzer(pwmController);
    modm::delay_ms(300);
    playNote(pwmController, 650);
    modm::delay_ms(150);
    silenceBuzzer(pwmController);
    modm::delay_ms(300);
    playNote(pwmController, 380);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(150);
    playNote(pwmController, 430);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(150);
    playNote(pwmController, 500);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(300);
    playNote(pwmController, 430);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(150);
    playNote(pwmController, 500);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(100);
    playNote(pwmController, 570);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(420);
    playNote(pwmController, 585);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(450);
    playNote(pwmController, 550);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(420);
    playNote(pwmController, 500);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(360);
    playNote(pwmController, 380);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(300);
    playNote(pwmController, 500);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(300);
    playNote(pwmController, 500);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(150);
    playNote(pwmController, 500);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(300);
    playNote(pwmController, 500);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(300);
    playNote(pwmController, 760);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(100);
    playNote(pwmController, 720);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(150);
    playNote(pwmController, 680);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(150);
    playNote(pwmController, 620);
    modm::delay_ms(150);
    silenceBuzzer(pwmController);
    modm::delay_ms(300);
    playNote(pwmController, 650);
    modm::delay_ms(150);
    silenceBuzzer(pwmController);
    modm::delay_ms(300);
    playNote(pwmController, 380);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(150);
    playNote(pwmController, 430);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(150);
    playNote(pwmController, 500);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(300);
    playNote(pwmController, 430);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(150);
    playNote(pwmController, 500);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(100);
    playNote(pwmController, 570);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(220);
    playNote(pwmController, 500);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(300);
    playNote(pwmController, 760);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(100);
    playNote(pwmController, 720);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(150);
    playNote(pwmController, 680);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(150);
    playNote(pwmController, 620);
    modm::delay_ms(150);
    silenceBuzzer(pwmController);
    modm::delay_ms(300);
    playNote(pwmController, 650);
    modm::delay_ms(200);
    silenceBuzzer(pwmController);
    modm::delay_ms(300);
    playNote(pwmController, 1020);
    modm::delay_ms(80);
    silenceBuzzer(pwmController);
    modm::delay_ms(300);
    playNote(pwmController, 1020);
    modm::delay_ms(80);
    silenceBuzzer(pwmController);
    modm::delay_ms(150);
    playNote(pwmController, 1020);
    modm::delay_ms(80);
    silenceBuzzer(pwmController);
    modm::delay_ms(300);
    playNote(pwmController, 380);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(300);
    playNote(pwmController, 500);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(300);
    playNote(pwmController, 760);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(100);
    playNote(pwmController, 720);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(150);
    playNote(pwmController, 680);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(150);
    playNote(pwmController, 620);
    modm::delay_ms(150);
    silenceBuzzer(pwmController);
    modm::delay_ms(300);
    playNote(pwmController, 650);
    modm::delay_ms(150);
    silenceBuzzer(pwmController);
    modm::delay_ms(300);
    playNote(pwmController, 380);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(150);
    playNote(pwmController, 430);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(150);
    playNote(pwmController, 500);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(300);
    playNote(pwmController, 430);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(150);
    playNote(pwmController, 500);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(100);
    playNote(pwmController, 570);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(420);
    playNote(pwmController, 585);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(450);
    playNote(pwmController, 550);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(420);
    playNote(pwmController, 500);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(360);
    playNote(pwmController, 380);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(300);
    playNote(pwmController, 500);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(300);
    playNote(pwmController, 500);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(150);
    playNote(pwmController, 500);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(300);
    playNote(pwmController, 500);
    modm::delay_ms(60);
    silenceBuzzer(pwmController);
    modm::delay_ms(150);
    playNote(pwmController, 500);
    modm::delay_ms(80);
    silenceBuzzer(pwmController);
    modm::delay_ms(300);
    playNote(pwmController, 500);
    modm::delay_ms(60);
    silenceBuzzer(pwmController);
    modm::delay_ms(350);
    playNote(pwmController, 500);
    modm::delay_ms(80);
    silenceBuzzer(pwmController);
    modm::delay_ms(150);
    playNote(pwmController, 580);
    modm::delay_ms(80);
    silenceBuzzer(pwmController);
    modm::delay_ms(350);
    playNote(pwmController, 660);
    modm::delay_ms(80);
    silenceBuzzer(pwmController);
    modm::delay_ms(150);
    playNote(pwmController, 500);
    modm::delay_ms(80);
    silenceBuzzer(pwmController);
    modm::delay_ms(300);
    playNote(pwmController, 430);
    modm::delay_ms(80);
    silenceBuzzer(pwmController);
    modm::delay_ms(150);
    playNote(pwmController, 380);
    modm::delay_ms(80);
    silenceBuzzer(pwmController);
    modm::delay_ms(600);
    playNote(pwmController, 500);
    modm::delay_ms(60);
    silenceBuzzer(pwmController);
    modm::delay_ms(150);
    playNote(pwmController, 500);
    modm::delay_ms(80);
    silenceBuzzer(pwmController);
    modm::delay_ms(300);
    playNote(pwmController, 500);
    modm::delay_ms(60);
    silenceBuzzer(pwmController);
    modm::delay_ms(350);
    playNote(pwmController, 500);
    modm::delay_ms(80);
    silenceBuzzer(pwmController);
    modm::delay_ms(150);
    playNote(pwmController, 580);
    modm::delay_ms(80);
    silenceBuzzer(pwmController);
    modm::delay_ms(150);
    playNote(pwmController, 660);
    modm::delay_ms(80);
    silenceBuzzer(pwmController);
    modm::delay_ms(550);
    playNote(pwmController, 870);
    modm::delay_ms(80);
    silenceBuzzer(pwmController);
    modm::delay_ms(325);
    playNote(pwmController, 760);
    modm::delay_ms(80);
    silenceBuzzer(pwmController);
    modm::delay_ms(600);
    playNote(pwmController, 500);
    modm::delay_ms(60);
    silenceBuzzer(pwmController);
    modm::delay_ms(150);
    playNote(pwmController, 500);
    modm::delay_ms(80);
    silenceBuzzer(pwmController);
    modm::delay_ms(300);
    playNote(pwmController, 500);
    modm::delay_ms(60);
    silenceBuzzer(pwmController);
    modm::delay_ms(350);
    playNote(pwmController, 500);
    modm::delay_ms(80);
    silenceBuzzer(pwmController);
    modm::delay_ms(150);
    playNote(pwmController, 580);
    modm::delay_ms(80);
    silenceBuzzer(pwmController);
    modm::delay_ms(350);
    playNote(pwmController, 660);
    modm::delay_ms(80);
    silenceBuzzer(pwmController);
    modm::delay_ms(150);
    playNote(pwmController, 500);
    modm::delay_ms(80);
    silenceBuzzer(pwmController);
    modm::delay_ms(300);
    playNote(pwmController, 430);
    modm::delay_ms(80);
    silenceBuzzer(pwmController);
    modm::delay_ms(150);
    playNote(pwmController, 380);
    modm::delay_ms(80);
    silenceBuzzer(pwmController);
    modm::delay_ms(600);
    playNote(pwmController, 660);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(150);
    playNote(pwmController, 660);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(300);
    playNote(pwmController, 660);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(300);
    playNote(pwmController, 510);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(100);
    playNote(pwmController, 660);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(300);
    playNote(pwmController, 770);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(550);
    playNote(pwmController, 380);
    modm::delay_ms(100);
    silenceBuzzer(pwmController);
    modm::delay_ms(575);
}


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
    aruwlib::Drivers *drivers = aruwlib::DoNotUse_getDrivers();

    Board::initialize();
    initializeIo(drivers);
    aruwsrc::control::initSubsystemCommands(drivers);

    playMarioThemesongBlocking(&drivers->pwm)

#ifdef PLATFORM_HOSTED
    aruwsrc::sim::initialize_robot_sim();
    aruwlib::motorsim::SimHandler::resetMotorSims();
    // Blocking call, waits until Windows Simulator connects.
    aruwlib::communication::TCPServer::MainServer()->getConnection();
#endif

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
            PROFILE(drivers->profiler, drivers->mpu6500.calcIMUAngles, ());
            PROFILE(drivers->profiler, drivers->errorController.updateLedDisplay, ());
            PROFILE(drivers->profiler, drivers->commandScheduler.run, ());
            PROFILE(drivers->profiler, drivers->djiMotorTxHandler.processCanSendData, ());
            PROFILE(drivers->profiler, drivers->terminalSerial.update, ());
            PROFILE(drivers->profiler, drivers->oledDisplay.updateMenu, ());
        }
        modm::delay_us(10);
    }
    return 0;
}

static void initializeIo(aruwlib::Drivers *drivers)
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
    drivers->xavierSerial.initializeCV();
#ifdef TARGET_SOLDIER
    drivers->imuRxHandler.init();
#endif
}

static void updateIo(aruwlib::Drivers *drivers)
{
#ifdef PLATFORM_HOSTED
    aruwlib::motorsim::SimHandler::updateSims();
#endif

    drivers->canRxHandler.pollCanData();
    drivers->refSerial.updateSerial();
    drivers->remote.read();
    drivers->oledDisplay.updateDisplay();
    drivers->mpu6500.read();
    drivers->xavierSerial.updateSerial();
}
