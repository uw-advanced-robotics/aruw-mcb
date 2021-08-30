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

#include "tap/rm-dev-board-a/board.hpp"

#include "modm/architecture/interface/delay.hpp"

/* arch includes ------------------------------------------------------------*/
#include "tap/architecture/periodic_timer.hpp"
#include "tap/architecture/profiler.hpp"

/* communication includes ---------------------------------------------------*/
#include "tap/drivers_singleton.hpp"

/* error handling includes --------------------------------------------------*/
#include "tap/errors/create_errors.hpp"

/* control includes ---------------------------------------------------------*/
#include "tap/algorithms/math_user_utils.hpp"
#include "tap/architecture/clock.hpp"

#include "aruwsrc/control/robot_control.hpp"
#include "aruwsrc/sim-initialization/robot_sim.hpp"

using tap::Drivers;

/* define timers here -------------------------------------------------------*/
tap::arch::PeriodicMilliTimer sendMotorTimeout(3);
tap::arch::PeriodicMilliTimer sendXavierTimeout(10);

// Place any sort of input/output initialization here. For example, place
// serial init stuff here.
static void initializeIo(tap::Drivers *drivers);

// Anything that you would like to be called place here. It will be called
// very frequently. Use PeriodicMilliTimers if you don't want something to be
// called as frequently.
static void updateIo(tap::Drivers *drivers);

// Testing temperature controller
static constexpr float TEMPERATURE_PID_P = 0.8f;
static constexpr float TEMPERATURE_PID_MAX_OUT = 0.9f;
static constexpr float HEATER_PWM_FREQUENCY = 1000.0f;
// Normal operating temperature is ~40 degrees C, and RM manual says the optimal operating
// temperature is ~15-20 degrees C above the normal operating temperature
static constexpr float TEMPERATURE_DESIRED_IMU = 55.0f;

void runTemperaturePController(tap::Drivers *drivers, float desiredTemperature)
{
    const float referenceTemperature = drivers->mpu6500.getTemp();

    // Run P controller to find desired output, units PWM frequency
    float out = TEMPERATURE_PID_P * (desiredTemperature - referenceTemperature);
    out = tap::algorithms::limitVal(out, 0.0f, TEMPERATURE_PID_MAX_OUT);

    // Debug output
    // drivers->terminalSerial.getStream()
    //     << out << "\t" << drivers->mpu6500.getTemp() << modm::endl;

    // Set heater PWM output
    drivers->pwm.write(out, tap::gpio::Pwm::HEATER);
}


bool calibrated = false;
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
    tap::Drivers *drivers = tap::DoNotUse_getDrivers();

    Board::initialize();
    initializeIo(drivers);
    aruwsrc::control::initSubsystemCommands(drivers);

#ifdef PLATFORM_HOSTED
    aruwsrc::sim::initialize_robot_sim();
    tap::motorsim::SimHandler::resetMotorSims();
    // Blocking call, waits until Windows Simulator connects.
    tap::communication::TCPServer::MainServer()->getConnection();
#endif

    drivers->pwm.setTimerFrequency(tap::gpio::Pwm::Timer::TIMER_3, HEATER_PWM_FREQUENCY);

    while (1)
    {
        // do this as fast as you can
        PROFILE(drivers->profiler, updateIo, (drivers));

        if (sendXavierTimeout.execute())
        {
            // PROFILE(drivers->profiler, drivers->xavierSerial.sendMessage, ());
            // TODO try faster baude rate so we can send more frequently (currently mcb's serial
            // buffers are overflowing if you try and send faster than 3 ms).
            // drivers->terminalSerial.getStream() << drivers->mpu6500.getYaw() << "\t" << (int)
            // drivers->mpu6500.getTemp() << modm::endl;
        }

        if (sendMotorTimeout.execute())
        {
            PROFILE(drivers->profiler, drivers->mpu6500.calcIMUAngles, ());
            runTemperaturePController(drivers, TEMPERATURE_DESIRED_IMU);
            if (!calibrated) {
                if (drivers->mpu6500.getTemp() > 54.44) {
                    drivers->mpu6500.calculateAccOffset();
                    drivers->mpu6500.calculateGyroOffset();
                }
                calibrated = true;
                drivers->terminalSerial.getStream() << "calibrated" << modm::endl;
            }
            drivers->terminalSerial.getStream() << tap::arch::clock::getTimeMilliseconds() << "\t" << drivers->mpu6500.getYaw() << modm::endl;
            // PROFILE(drivers->profiler, drivers->errorController.updateLedDisplay, ());
            // PROFILE(drivers->profiler, drivers->commandScheduler.run, ());
            // PROFILE(drivers->profiler, drivers->djiMotorTxHandler.processCanSendData, ());
            // PROFILE(drivers->profiler, drivers->terminalSerial.update, ());
            // PROFILE(drivers->profiler, drivers->oledDisplay.updateMenu, ());
        }
        modm::delay_us(10);
    }
    return 0;
}

static void initializeIo(tap::Drivers *drivers)
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
    // drivers->xavierSerial.initializeCV();
#ifdef TARGET_SOLDIER
    drivers->imuRxHandler.init();
#endif
}

static void updateIo(tap::Drivers *drivers)
{
#ifdef PLATFORM_HOSTED
    tap::motorsim::SimHandler::updateSims();
#endif

    drivers->canRxHandler.pollCanData();
    drivers->refSerial.updateSerial();
    drivers->remote.read();
    drivers->oledDisplay.updateDisplay();
    drivers->mpu6500.read();
    // drivers->xavierSerial.updateSerial();
}
