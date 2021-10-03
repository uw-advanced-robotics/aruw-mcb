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

/* control includes ---------------------------------------------------------*/
#include "tap/architecture/clock.hpp"
#include "tap/architecture/endianness_wrappers.hpp"

#include "aruwsrc/control/hopper-cover/hopper_subsystem.hpp"
#include "aruwsrc/control/hopper-cover/open_hopper_command_old.hpp"
#include "modm/architecture/interface/can_message.hpp"

using tap::Drivers;

static constexpr int16_t MIN_FLYWHEEL_SPEED_FLYWHEELS_OFF = 100;

static constexpr tap::motor::MotorId LEFT_MOTOR_ID = tap::motor::MOTOR2;
static constexpr tap::motor::MotorId RIGHT_MOTOR_ID = tap::motor::MOTOR1;
static constexpr tap::can::CanBus CAN_BUS_MOTORS = tap::can::CanBus::CAN_BUS1;

tap::motor::DjiMotor flywheelLeft(
    tap::DoNotUse_getDrivers(),
    LEFT_MOTOR_ID,
    CAN_BUS_MOTORS,
    false,
    "flywheel left");
tap::motor::DjiMotor flywheelRight(
    tap::DoNotUse_getDrivers(),
    RIGHT_MOTOR_ID,
    CAN_BUS_MOTORS,
    false,
    "flywheel right");

aruwsrc::control::HopperSubsystem hopperSubsystem(
    tap::DoNotUse_getDrivers(),
    tap::gpio::Pwm::Pin::W,
    aruwsrc::control::HopperSubsystem::SOLDIER_HOPPER_OPEN_PWM,
    aruwsrc::control::HopperSubsystem::SOLDIER_HOPPER_CLOSE_PWM,
    aruwsrc::control::HopperSubsystem::SOLDIER_PWM_RAMP_SPEED);

aruwsrc::control::OpenHopperCommand openHopper(&hopperSubsystem);

bool flywheelsOn = false;
bool prevFlywheelsOn = true;

int blinkCounter = 0;

tap::arch::PeriodicMilliTimer mainLoopTimeout(2);

// Place any sort of input/output initialization here. For example, place
// serial init stuff here.
static void initializeIo(tap::Drivers *drivers);

// Anything that you would like to be called place here. It will be called
// very frequently. Use PeriodicMilliTimers if you don't want something to be
// called as frequently.
static void updateIo(tap::Drivers *drivers);

static void sendIMUData(tap::Drivers *drivers);
static void updateFlywheelState(tap::Drivers *drivers);
static void updateHopperCover(tap::Drivers *drivers);
static void updateLaser(tap::Drivers *drivers);

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

    flywheelLeft.initialize();
    flywheelRight.initialize();

    drivers->commandScheduler.registerSubsystem(&hopperSubsystem);

    while (1)
    {
        // do this as fast as you can
        PROFILE(drivers->profiler, updateIo, (drivers));

        if (mainLoopTimeout.execute())
        {
            PROFILE(drivers->profiler, drivers->mpu6500.periodicIMUUpdate, ());
            PROFILE(drivers->profiler, drivers->terminalSerial.update, ());
            PROFILE(drivers->profiler, drivers->commandScheduler.run, ());

            sendIMUData(drivers);
            updateFlywheelState(drivers);
            updateHopperCover(drivers);
            updateLaser(drivers);
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
    drivers->errorController.init();
    drivers->mpu6500TerminalSerialHandler.init();
    drivers->terminalSerial.initialize();
}

static void updateIo(tap::Drivers *drivers)
{
    drivers->mpu6500.read();
    drivers->canRxHandler.pollCanData();
}

static void updateFlywheelState(tap::Drivers *drivers)
{
    prevFlywheelsOn = flywheelsOn;
    flywheelsOn = abs(flywheelLeft.getShaftRPM()) > MIN_FLYWHEEL_SPEED_FLYWHEELS_OFF &&
                  abs(flywheelRight.getShaftRPM()) > MIN_FLYWHEEL_SPEED_FLYWHEELS_OFF;
}

static void sendIMUData(tap::Drivers *drivers)
{
    float yaw = drivers->mpu6500.getYaw();
    int16_t gzRaw = drivers->mpu6500.getGz() * tap::sensors::Mpu6500::LSB_D_PER_S_TO_D_PER_S;

    if (drivers->can.isReadyToSend(tap::can::CanBus::CAN_BUS1))
    {
        drivers->leds.set(tap::gpio::Leds::Green, blinkCounter < 50);
        blinkCounter = (blinkCounter + 1) % 100;

        modm::can::Message msg(IMU_MSG_CAN_ID, 8);
        msg.setExtended(false);
        tap::arch::convertToLittleEndian(yaw, msg.data);
        tap::arch::convertToLittleEndian(gzRaw, msg.data + 4);
        drivers->can.sendMessage(tap::can::CanBus::CAN_BUS1, msg);
    }
}

static void updateHopperCover(tap::Drivers *drivers)
{
    if (!flywheelsOn && prevFlywheelsOn)
    {
        drivers->commandScheduler.addCommand(&openHopper);
    }
    else if (flywheelsOn && !prevFlywheelsOn)
    {
        drivers->commandScheduler.removeCommand(&openHopper, false);
    }
}

static void updateLaser(tap::Drivers *drivers)
{
    if (!flywheelsOn && prevFlywheelsOn)
    {
        drivers->digital.set(tap::gpio::Digital::OutputPin::Laser, false);
    }
    else if (flywheelsOn && !prevFlywheelsOn)
    {
        drivers->digital.set(tap::gpio::Digital::OutputPin::Laser, true);
    }
}
