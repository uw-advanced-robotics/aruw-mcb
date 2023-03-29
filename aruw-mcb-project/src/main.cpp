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

#include "aruwsrc/robot/robot_control.hpp"
#include "aruwsrc/sim-initialization/robot_sim.hpp"
#include "aruwsrc/util_macros.hpp"


static constexpr float MAIN_LOOP_FREQUENCY = 500.0f;
static constexpr float MAHONY_KP = 0.1f;


// for looking at our odometry values
#include "modm/math/geometry/vector3.hpp"

float chassisXS = 5.0;
float chassisYS = 5.0;
float chassisZS = 5.0;

float chassisRollS = 0.0;
float chassisPitchS = 0.0;
float chassisYawS = 0.0;

float turretRollS = 0.0;
float turretPitchS = 0.0;
float turretYawS = 0.0;


float chassisXO = 0.0;
float chassisYO = 0.0;

float chassisXin = 0.f;
float chassisYin = 0.f;


// static float chassisZO = 0.0;

// static float chassisRollO = 0.0;
// static float chassisPitchO = 0.0;
float chassisYawO = 0.0;

// static float turretRollO = 0.0;
// static float turretPitchO = 0.0;
// static float turretYawO = 0.0;

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
#elif defined(TARGET_HERO_CYCLONE)
using namespace aruwsrc::hero;
#elif defined(TARGET_DRONE)
using namespace aruwsrc::drone;
#elif defined(TARGET_ENGINEER)
using namespace aruwsrc::engineer;
#endif

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
        modm::Vector3f& chassisWorldPosition = drivers->transformer.chassisWorldPosition;
        modm::Vector3f& chassisWorldOrientation = drivers->transformer.chassisWorldOrientation;
        modm::Vector3f& turretWorldOrientation = drivers->transformer.turretWorldOrientation;

        chassisXS = chassisWorldPosition.getX();
        chassisYS = chassisWorldPosition.getY();
        chassisZS = chassisWorldPosition.getZ();

        chassisXin = drivers->transformer.vel_x_in;
        chassisXin = drivers->transformer.nextKFInput[2];

        // chassisXS = 5.0f;

        chassisRollS = chassisWorldOrientation.getX();
        chassisPitchS = chassisWorldOrientation.getY();
        chassisYawS = chassisWorldOrientation.getZ();

        turretRollS = turretWorldOrientation.getX();
        turretPitchS = turretWorldOrientation.getY();
        turretYawS = turretWorldOrientation.getZ();

        // TODO: get odometry values
        // in odometry subsystem we need to get public references to 
        // stored odoemstry stuff
        
        modm::Location2D<float> odomLoc = drivers->removeThisOdom->getCurrentLocation2D();
        chassisXO = odomLoc.getX();
        chassisYO = odomLoc.getY();
 
 
        chassisYawO = drivers->removeThisOdom->getYaw();

        // do this as fast as you can
        PROFILE(drivers->profiler, updateIo, (drivers));

        if (sendMotorTimeout.execute())
        {
            PROFILE(drivers->profiler, drivers->mpu6500.periodicIMUUpdate, ());
            PROFILE(drivers->profiler, drivers->commandScheduler.run, ());
            PROFILE(drivers->profiler, drivers->djiMotorTxHandler.encodeAndSendCanData, ());
            PROFILE(drivers->profiler, drivers->terminalSerial.update, ());

#if defined(ALL_STANDARDS) || defined(TARGET_HERO_CYCLONE) || defined(TARGET_SENTRY_BEEHIVE)
            PROFILE(drivers->profiler, drivers->oledDisplay.updateMenu, ());
#endif

#if defined(ALL_STANDARDS) || defined(TARGET_HERO_CYCLONE) || defined(TARGET_SENTRY_BEEHIVE)
            PROFILE(drivers->profiler, drivers->turretMCBCanCommBus1.sendData, ());
#endif

#if defined(TARGET_SENTRY_BEEHIVE)
            PROFILE(drivers->profiler, drivers->turretMCBCanCommBus2.sendData, ());
#endif

#if defined(ALL_STANDARDS) || defined(TARGET_HERO_CYCLONE) || defined(TARGET_SENTRY_BEEHIVE)
            PROFILE(drivers->profiler, drivers->visionCoprocessor.sendMessage, ());
#endif

#if defined(ALL_STANDARDS)
            PROFILE(drivers->profiler, drivers->transformer.update, ());
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
    drivers->terminalSerial.initialize();
    drivers->schedulerTerminalHandler.init();
    drivers->djiMotorTerminalSerialHandler.init();

#if defined(TARGET_HERO_CYCLONE) || defined(ALL_STANDARDS) || defined(TARGET_SENTRY_BEEHIVE)
    ((Drivers *)drivers)->visionCoprocessor.initializeCV();
    ((Drivers *)drivers)->mpu6500TerminalSerialHandler.init();
    ((Drivers *)drivers)->turretMCBCanCommBus1.init();
    ((Drivers *)drivers)->oledDisplay.initialize();
#endif
}

static void updateIo(tap::Drivers *drivers)
{
    drivers->canRxHandler.pollCanData();
    drivers->refSerial.updateSerial();
    drivers->remote.read();
    drivers->mpu6500.read();

#ifdef ALL_STANDARDS
    ((Drivers *)drivers)->oledDisplay.updateDisplay();
    ((Drivers *)drivers)->visionCoprocessor.updateSerial();
#endif
#ifdef TARGET_HERO_CYCLONE
    ((Drivers *)drivers)->oledDisplay.updateDisplay();
    ((Drivers *)drivers)->visionCoprocessor.updateSerial();
#endif
#ifdef TARGET_SENTRY_BEEHIVE
    ((Drivers *)drivers)->oledDisplay.updateDisplay();
    ((Drivers *)drivers)->visionCoprocessor.updateSerial();
#endif
}
