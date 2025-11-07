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
#include "modm/platform/rtt/rtt.hpp"

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

#include "aruwsrc/control/chassis/constants/chassis_constants.hpp"
#include "aruwsrc/robot/robot_control.hpp"
#include "aruwsrc/sim-initialization/robot_sim.hpp"
#include "aruwsrc/util_macros.hpp"

static constexpr float MAIN_LOOP_FREQUENCY = 500.0f;
static constexpr float MAHONY_KP = 0.1f;

/* define timers here -------------------------------------------------------*/
tap::arch::PeriodicMilliTimer sendMotorTimeout(1000.0f / MAIN_LOOP_FREQUENCY);

#if defined(ALL_STANDARDS)
using namespace aruwsrc::standard;
#elif defined(ALL_SENTRIES)
using namespace aruwsrc::sentry;
#elif defined(TARGET_HERO_ZERO)
using namespace aruwsrc::hero;
#elif defined(TARGET_DRONE)
using namespace aruwsrc::drone;
#elif defined(TARGET_ENGINEER)
using namespace aruwsrc::engineer;
#elif defined(TARGET_DART)
using namespace aruwsrc::dart;
#elif defined(TARGET_TESTBED)
using namespace aruwsrc::testbed;
#elif defined(TARGET_BLANK)
using namespace aruwsrc::blank;
#elif defined(TARGET_MOTOR_TESTER)
using namespace aruwsrc::motor_tester;
#elif defined(TARGET_CHARACTERIZER)
using namespace aruwsrc::characterizer;
#endif

// Place any sort of input/output initialization here. For example, place
// serial init stuff here.
static void initializeIo(Drivers* drivers);

// Anything that you would like to be called place here. It will be called
// very frequently. Use PeriodicMilliTimers if you don't want something to be
// called as frequently.
static void updateIo(Drivers* drivers);

static void initializeI2C(Drivers* drivers);

#if defined(ALL_STANDARDS) || defined(TARGET_HERO_ZERO)
// Check if the turret MCB on CAN 1 is disconnected and sounds buzzer if it is
static void checkTurretMcbDisconnection(Drivers* drivers);
#endif

namespace
{
Drivers* driversForAssert = nullptr;
}

/*
#ifdef ALL_STANDARDS  // temp, bc logging only added for standard atm
static modm::Abandonment log_assertion(const modm::AssertionInfo& info)
{
    if (driversForAssert) driversForAssert->rttTelemetry.println("Assertion raised: ", info.name);
    return modm::Abandonment::DontCare;
}
MODM_ASSERTION_HANDLER(log_assertion);

modm_extern_c modm_noreturn void modm_abandon(const modm::AssertionInfo& info)
{
    if (driversForAssert)
    {
        driversForAssert->rttTelemetry.println("ABORTING - Assertion raised: ", info.name);

        driversForAssert->rttTelemetry.sendQueuedMessages();
        modm::delay_ms(1);  // maybe unnecessary / too long
    }
}
#endif
*/

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
    Drivers* drivers = DoNotUse_getDrivers();
    driversForAssert = drivers;

    Board::initialize();
    initializeIo(drivers);
    initSubsystemCommands(drivers);

    while (1)
    {
        // do this as fast as you can
        PROFILE(drivers->profiler, updateIo, (drivers));

        if (sendMotorTimeout.execute())
        {
            PROFILE(drivers->profiler, drivers->mpu6500.periodicIMUUpdate, ());
            PROFILE(drivers->profiler, drivers->commandScheduler.run, ());
            PROFILE(drivers->profiler, drivers->djiMotorTxHandler.encodeAndSendCanData, ());

#if defined(ALL_STANDARDS) || defined(TARGET_HERO_ZERO) || defined(TARGET_SENTRY_ECLIPSE)
            PROFILE(drivers->profiler, drivers->oledDisplay.updateMenu, ());
            ((Drivers*)drivers)->plateHitTracker.update();
#endif

#if defined(ALL_STANDARDS) || defined(TARGET_HERO_ZERO) || defined(TARGET_SENTRY_ECLIPSE)
            PROFILE(drivers->profiler, drivers->turretMCBCanCommBus1.sendData, ());
#endif

#if defined(TARGET_ENGINEER)
            PROFILE(drivers->profiler, drivers->oledDisplay.updateMenu, ());
#endif

#if defined(TARGET_SENTRY_ECLIPSE)
            PROFILE(drivers->profiler, drivers->turretMCBCanCommBus2.sendData, ());
            PROFILE(drivers->profiler, drivers->chassisMcbLite.sendData, ());
            PROFILE(drivers->profiler, drivers->turretMajorImu.periodicIMUUpdate, ());
#endif

#ifdef TARGET_TESTBED
            PROFILE(drivers->profiler, drivers->lite.sendData, ());
#endif

#if defined(ALL_STANDARDS) || defined(TARGET_HERO_ZERO) || defined(TARGET_SENTRY_ECLIPSE)
            PROFILE(drivers->profiler, drivers->visionCoprocessor.sendMessage, ());
#endif

#if defined(ALL_STANDARDS) || defined(TARGET_HERO_ZERO)
            checkTurretMcbDisconnection(drivers);
#endif

#if defined(ALL_STANDARDS) || defined(TARGET_MOTOR_TESTER)
#if !defined(PLATFORM_HOSTED) || !defined(ENV_UNIT_TESTS)
            PROFILE(drivers->profiler, ((Drivers*)drivers)->rttTelemetry.updateTelemetryAsync, ());
#endif
#endif

#if defined(ALL_STANDARDS) || defined(TARGET_HERO_ZERO)
            // PROFILE(drivers->profiler, drivers->ism330.periodicIMUUpdate, ());
#endif
        }
        modm::delay_us(10);
    }
    return 0;
}

static void initializeIo(Drivers* drivers)
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

    initializeI2C(drivers);

#if defined(TARGET_HERO_ZERO) || defined(ALL_STANDARDS) || defined(TARGET_SENTRY_ECLIPSE)
    drivers->visionCoprocessor.initializeCV();
    drivers->turretMCBCanCommBus1.init();
#endif
#if defined(TARGET_HERO_ZERO) || defined(ALL_STANDARDS) || defined(TARGET_SENTRY_ECLIPSE) || \
    defined(TARGET_ENGINEER)
    ((Drivers*)drivers)->oledDisplay.initialize();
#endif
#if defined(TARGET_HERO_ZERO) || defined(ALL_STANDARDS)
    drivers->mpu6500.setCalibrationSamples(2000);
#endif
#if defined(TARGET_HERO_ZERO) || defined(ALL_STANDARDS)
    ((Drivers*)drivers)->capacitorBank.initialize();
#endif
#if defined(TARGET_SENTRY_ECLIPSE)
    drivers->turretMCBCanCommBus2.init();
    // Needs to be same time period as the calibration period of the minors and mcb-lite is as this
    // dictates command length
    drivers->mpu6500.setCalibrationSamples(4000);
    drivers->chassisMcbLite.initialize();
    modm::delay_ms(2000);
    drivers->turretMajorImu.initialize(MAIN_LOOP_FREQUENCY, MAHONY_KP, 0.0f);
    drivers->turretMajorImu.setCalibrationSamples(4000);
#endif
#ifdef TARGET_TESTBED
    drivers->lite.initialize();
#endif
#if defined(TARGET_ENGINEER)
    drivers->engineerCVCommunication.initializeCV();
#endif

#if defined(ALL_STANDARDS) || defined(TARGET_HERO_ZERO)
    // modm::delay_ms(2000);
    // drivers->ism330.initialize(MAIN_LOOP_FREQUENCY, MAHONY_KP, 0.0f);
#endif
}

static void updateIo(Drivers* drivers)
{
    drivers->canRxHandler.pollCanData();
    drivers->refSerial.updateSerial();
    drivers->remote.read();
    drivers->mpu6500.read();

#if defined(ALL_STANDARDS) || defined(TARGET_HERO_ZERO) || defined(TARGET_SENTRY_ECLIPSE) || \
    defined(TARGET_ENGINEER)
    ((Drivers*)drivers)->oledDisplay.updateDisplay();
#endif

#if defined(ALL_STANDARDS) || defined(TARGET_HERO_ZERO) || defined(TARGET_SENTRY_ECLIPSE)
    drivers->visionCoprocessor.updateSerial();
#endif

#ifdef TARGET_ENGINEER
    drivers->engineerCVCommunication.updateSerial();
#endif

#ifdef TARGET_SENTRY_ECLIPSE
    drivers->chassisMcbLite.updateSerial();
    drivers->turretMajorImu.read();
#endif

#ifdef TARGET_TESTBED
    drivers->lite.updateSerial();
#endif

#if defined(TARGET_HERO_ZERO) || defined(ALL_STANDARDS)
    drivers->interRobotTransmitter.updateState();
    drivers->interRobotTransmitter.sendMessage();
#endif

#if defined(ALL_STANDARDS) || defined(TARGET_HERO_ZERO)
    // drivers->ism330.read();
#endif

#if defined(TARGET_SENTRY_ECLIPSE)
    drivers->stateMachine.updateState();
#endif
}

#if defined(ALL_STANDARDS) || defined(TARGET_HERO_ZERO)
static void checkTurretMcbDisconnection(Drivers* drivers)
{
    bool turretMcbConnected = drivers->turretMCBCanCommBus1.isConnected();
    if (!turretMcbConnected &&
        drivers->mpu6500.getImuState() !=
            tap::communication::sensors::imu::ImuInterface::ImuState::IMU_CALIBRATING)
    {
        tap::buzzer::playNote(&drivers->pwm, 1000);
    }
    else
    {
        tap::buzzer::silenceBuzzer(&drivers->pwm);
    }
}
#endif

static void initializeI2C(Drivers* drivers)
{
    drivers->digital.set(tap::gpio::Digital::OutputPin::E, true);
    modm::delay_ms(2000);  // Wait for the SDA and SCL lines to be pulled high

    Board::I2CMaster::connect<Board::I2cScl::Scl, Board::I2CSda::Sda>(
        Board::I2CMaster::PullUps::External);
    Board::I2CMaster::initialize<Board::SystemClock, 300'000>();
    Board::I2CMaster::reset();
}
