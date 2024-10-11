/*
 * Copyright (c) 2020-2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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
#include "tap/communication/sensors/buzzer/buzzer.hpp"

#include "aruwsrc/robot/robot_control.hpp"
#include "aruwsrc/sim-initialization/robot_sim.hpp"
#include "aruwsrc/util_macros.hpp"

static constexpr float MAIN_LOOP_FREQUENCY = 500.0f;
static constexpr float MAHONY_KP = 0.1f;

/* define timers here -------------------------------------------------------*/
tap::arch::PeriodicMilliTimer sendMotorTimeout(1000.0f / MAIN_LOOP_FREQUENCY);

// Place any sort of input/output initialization here. For example, place
// serial init stuff here.
static void initializeIo(tap::Drivers *drivers);

// Anything that you would like to be called place here. It will be called
// very frequently. Use PeriodicMilliTimers if you don't want something to be
// called as frequently.
static void updateIo(tap::Drivers *drivers);

#if defined(ALL_STANDARDS)
using namespace aruwsrc::standard;
#elif defined(ALL_SENTRIES)
using namespace aruwsrc::sentry;
#elif defined(TARGET_HERO_PERSEUS)
using namespace aruwsrc::hero;
#elif defined(TARGET_DRONE)
using namespace aruwsrc::drone;
#elif defined(TARGET_ENGINEER)
using namespace aruwsrc::engineer;
#elif defined(TARGET_DART)
using namespace aruwsrc::dart;
#elif defined(TARGET_TESTBED)
using namespace aruwsrc::testbed;
#endif
float temp = 0.0f;
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
    Drivers *drivers = DoNotUse_getDrivers();

    Board::initialize();
    initializeIo(drivers);
    initSubsystemCommands(drivers);

    while (1)
    {
        // do this as fast as you can
        PROFILE(drivers->profiler, updateIo, (drivers));

        if (sendMotorTimeout.execute())
        {
            temp = drivers->mpu6500.getTemp();
            PROFILE(drivers->profiler, drivers->mpu6500.periodicIMUUpdate, ());
            PROFILE(drivers->profiler, drivers->commandScheduler.run, ());
            PROFILE(drivers->profiler, drivers->djiMotorTxHandler.encodeAndSendCanData, ());

#if defined(ALL_STANDARDS) || defined(TARGET_HERO_PERSEUS) || defined(TARGET_SENTRY_HYDRA)
            PROFILE(drivers->profiler, drivers->oledDisplay.updateMenu, ());
#endif

#if defined(ALL_STANDARDS) || defined(TARGET_HERO_PERSEUS) || defined(TARGET_SENTRY_HYDRA)
            PROFILE(drivers->profiler, drivers->turretMCBCanCommBus1.sendData, ());
#endif

#if defined(TARGET_SENTRY_HYDRA)
            PROFILE(drivers->profiler, drivers->turretMCBCanCommBus2.sendData, ());
            PROFILE(drivers->profiler, drivers->chassisMcbLite.sendData, ());
            PROFILE(drivers->profiler, drivers->turretMajorMcbLite.sendData, ());
#endif

#if defined(ALL_STANDARDS) || defined(TARGET_HERO_PERSEUS) || defined(TARGET_SENTRY_HYDRA)
            PROFILE(drivers->profiler, drivers->visionCoprocessor.sendMessage, ());
#endif

#if defined(ALL_STANDARDS) || defined(TARGET_HERO_PERSEUS)
            bool turretMcbConnected = drivers->turretMCBCanCommBus1.isConnected();
            if (!turretMcbConnected)
            {
                tap::buzzer::playNote(&drivers->pwm, 1000);
            }
            else
            {
                tap::buzzer::silenceBuzzer(&drivers->pwm);
            }
#endif
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
    drivers->mpu6500.init(MAIN_LOOP_FREQUENCY, MAHONY_KP, 0.0f);
    drivers->refSerial.initialize();

#if defined(TARGET_HERO_PERSEUS) || defined(ALL_STANDARDS) || defined(TARGET_SENTRY_HYDRA)
    ((Drivers *)drivers)->visionCoprocessor.initializeCV();
    ((Drivers *)drivers)->turretMCBCanCommBus1.init();
#endif
#if defined(TARGET_HERO_PERSEUS) || defined(ALL_STANDARDS) || defined(TARGET_SENTRY_HYDRA)
    ((Drivers *)drivers)->oledDisplay.initialize();
#endif
#if defined(TARGET_HERO_PERSEUS) || defined(ALL_STANDARDS)
    ((Drivers *)drivers)->mpu6500.setCalibrationSamples(2000);
#endif
#if defined(TARGET_HERO_PERSEUS) || defined(ALL_STANDARDS) || defined(TARGET_TESTBED)
    ((Drivers *)drivers)->capacitorBank.initialize();
#endif
#if defined(TARGET_HERO_PERSEUS) || defined(ALL_STANDARDS) || defined(TARGET_TESTBED) || \
    defined(TARGET_SENTRY_HYDRA)
    ((Drivers *)drivers)->plateHitTracker.initialize();
#endif
#if defined(TARGET_SENTRY_HYDRA)
    ((Drivers *)drivers)->turretMCBCanCommBus2.init();
    // Needs to be same time period as the calibration period of the minors and mcb-lite is as this
    // dictates command length
    ((Drivers *)drivers)->mpu6500.setCalibrationSamples(4000);
    ((Drivers *)drivers)->chassisMcbLite.initialize();
    ((Drivers *)drivers)->turretMajorMcbLite.initialize();
#endif
}

static void updateIo(tap::Drivers *drivers)
{
    drivers->canRxHandler.pollCanData();
    drivers->refSerial.updateSerial();
    drivers->remote.read();
    drivers->mpu6500.read();

#if defined(ALL_STANDARDS) || defined(TARGET_HERO_PERSEUS) || defined(TARGET_SENTRY_HYDRA)
    ((Drivers *)drivers)->oledDisplay.updateDisplay();
    ((Drivers *)drivers)->plateHitTracker.update();
    ((Drivers *)drivers)->plateHitTracker.getPeakAngleDegrees();  // @todo remove this
#endif

#ifdef ALL_STANDARDS
    ((Drivers *)drivers)->visionCoprocessor.updateSerial();
#endif
#ifdef TARGET_HERO_PERSEUS
    ((Drivers *)drivers)->visionCoprocessor.updateSerial();
#endif
#ifdef TARGET_SENTRY_HYDRA
    ((Drivers *)drivers)->chassisMcbLite.updateSerial();
    ((Drivers *)drivers)->turretMajorMcbLite.updateSerial();
    ((Drivers *)drivers)->visionCoprocessor.updateSerial();
#endif
}
