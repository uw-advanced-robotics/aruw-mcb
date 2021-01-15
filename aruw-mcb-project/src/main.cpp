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
#include <iostream>
#endif

#include <aruwlib/rm-dev-board-a/board.hpp>
#include <modm/platform/core/delay.hpp>

/* arch includes ------------------------------------------------------------*/
#include <aruwlib/architecture/periodic_timer.hpp>

/* communication includes ---------------------------------------------------*/
#include <aruwlib/DriversSingleton.hpp>
#include <aruwlib/display/sh1106.hpp>

/* error handling includes --------------------------------------------------*/

/* control includes ---------------------------------------------------------*/
#include "aruwsrc/control/robot_control.hpp"

#include "matplotlibcpp.h"
#include "two_dim_filter.hpp"

namespace plt = matplotlibcpp;
using namespace modm::literals;
using aruwlib::Drivers;

/* define timers here -------------------------------------------------------*/
aruwlib::arch::PeriodicMilliTimer sendMotorTimeout(2);

// Place any sort of input/output initialization here. For example, place
// serial init stuff here.
void initializeIo(aruwlib::Drivers *drivers);

// Anything that you would like to be called place here. It will be called
// very frequently. Use PeriodicMilliTimers if you don't want something to be
// called as frequently.
void updateIo(aruwlib::Drivers *drivers);

int main()
{
#ifdef PLATFORM_HOSTED
    std::cout << "Simulation starting..." << std::endl;
#endif
    TwoDimFilter f;
    float vxWheels = 1000;
    float vyWheels = 1000;
    float yawIMU = 0;
    float omegaWheels = 0;
    float omegaIMU = 0;
    std::vector<float> xPosition;
    for (int i = 0; i < 500; i++)
    {
        if (yawIMU < aruwlib::algorithms::PI / 2) {
            omegaWheels = aruwlib::algorithms::PI / 0.064;
            omegaIMU = aruwlib::algorithms::PI / 0.064;
            yawIMU += aruwlib::algorithms::PI / 32;
        } else {
            omegaWheels = 0;
            omegaIMU = 0;
        }
        f.update(vxWheels, vyWheels, yawIMU, omegaWheels, omegaIMU);
        const modm::Matrix<float, 6, 1> &x = f.getX();
        xPosition.push_back(x[5][0]);
    }
    plt::plot(xPosition);
    plt::show();

    return 0;
}

void initializeIo(aruwlib::Drivers *drivers)
{
    drivers->analog.init();
    drivers->pwm.init();
    drivers->digital.init();
    drivers->leds.init();
    drivers->can.initialize();

#ifndef PLATFORM_HOSTED
    /// \todo this should be an init in the display class
    Board::DisplaySpiMaster::
        connect<Board::DisplayMiso::Miso, Board::DisplayMosi::Mosi, Board::DisplaySck::Sck>();

    // SPI1 is on ABP2 which is at 90MHz; use prescaler 64 to get ~fastest baud rate below 1mHz max
    // 90MHz/64=~14MHz
    Board::DisplaySpiMaster::initialize<Board::SystemClock, 1406250_Hz>();
#endif
    aruwlib::display::Sh1106<
#ifndef PLATFORM_HOSTED
        Board::DisplaySpiMaster,
        Board::DisplayCommand,
        Board::DisplayReset,
#endif
        128,
        64,
        false>
        display;
    display.initializeBlocking();

    drivers->remote.initialize();
    drivers->mpu6500.init();
    drivers->refSerial.initialize();
    drivers->xavierSerial.initialize();
}

void updateIo(aruwlib::Drivers *drivers)
{
    drivers->canRxHandler.pollCanData();
    drivers->xavierSerial.updateSerial();
    drivers->refSerial.updateSerial();
    drivers->remote.read();
}
