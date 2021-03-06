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

#include <aruwlib/DriversSingleton.hpp>
#include <aruwlib/communication/gpio/digital.hpp>
#include <aruwlib/control/command_scheduler.hpp>

#include "aruwsrc/control/agitator/agitator_calibrate_command.hpp"
#include "aruwsrc/control/agitator/agitator_shoot_comprised_command.hpp"
#include "aruwsrc/control/agitator/agitator_subsystem.hpp"
#include "aruwsrc/control/engineer/EngineerWristCalibrateCommand.hpp"
#include "aruwsrc/control/engineer/EngineerWristRotateCommand.hpp"
#include "aruwsrc/control/engineer/EngineerWristSubsystem.hpp"
#include "aruwsrc/control/engineer/TowSubsystem.hpp"
#include "aruwsrc/control/engineer/extend_xaxis_command.hpp"
#include "aruwsrc/control/engineer/grabber_subsystem.hpp"
#include "aruwsrc/control/engineer/squeeze_grabber_command.hpp"
#include "aruwsrc/control/engineer/xaxis_subsystem.hpp"

#if defined(TARGET_ENGINEER)

using namespace aruwsrc::engineer;
using namespace aruwsrc::agitator;
using namespace aruwlib::gpio;
using aruwlib::DoNotUse_getDrivers;
using aruwlib::control::CommandMapper;

/*
 * NOTE: We are using the DoNotUse_getDrivers() function here
 *      because this file defines all subsystems and command
 *      and thus we must pass in the single statically allocated
 *      Drivers class to all of these objects.
 */
aruwlib::driversFunc drivers = aruwlib::DoNotUse_getDrivers;

namespace aruwsrc
{
namespace control
{
static constexpr Digital::OutputPin GRABBER_PIN = Digital::OutputPin::E;
static constexpr Digital::OutputPin X_AXIS_PIN = Digital::OutputPin::F;
static constexpr Digital::OutputPin TOWER_LEFT_PIN = Digital::OutputPin::G;
static constexpr Digital::OutputPin TOWER_RIGHT_PIN = Digital::OutputPin::H;
static constexpr Digital::InputPin TOWER_LEFT_LIMIT_SWITCH = Digital::InputPin::A;
static constexpr Digital::InputPin TOWER_RIGHT_LIMIT_SWITCH = Digital::InputPin::B;

/* define subsystems --------------------------------------------------------*/
GrabberSubsystem grabber(drivers(), GRABBER_PIN);
XAxisSubsystem xAxis(drivers(), X_AXIS_PIN);
TowSubsystem tower(
    drivers(),
    TOWER_LEFT_PIN,
    TOWER_RIGHT_PIN,
    TOWER_LEFT_LIMIT_SWITCH,
    TOWER_RIGHT_LIMIT_SWITCH);
AgitatorSubsystem reservoir17mm(
    drivers(),
    AgitatorSubsystem::PID_RESERVOIR_17MM_P,
    AgitatorSubsystem::PID_RESERVOIR_17MM_I,
    AgitatorSubsystem::PID_RESERVOIR_17MM_D,
    AgitatorSubsystem::PID_RESERVOIR_17MM_MAX_ERR_SUM,
    AgitatorSubsystem::PID_RESERVOIR_17MM_MAX_OUT,
    AgitatorSubsystem::AGITATOR_GEAR_RATIO_GM3508,
    AgitatorSubsystem::RESERVOIR_17MM_MOTOR_ID,
    AgitatorSubsystem::RESERVOIR_17MM_MOTOR_CAN_BUS,
    AgitatorSubsystem::RESERVOIR_17MM_INVERTED);
AgitatorSubsystem reservoir42mm(
    drivers(),
    AgitatorSubsystem::PID_RESERVOIR_42MM_P,
    AgitatorSubsystem::PID_RESERVOIR_42MM_I,
    AgitatorSubsystem::PID_RESERVOIR_42MM_D,
    AgitatorSubsystem::PID_RESERVOIR_42MM_MAX_ERR_SUM,
    AgitatorSubsystem::PID_RESERVOIR_42MM_MAX_OUT,
    AgitatorSubsystem::AGITATOR_GEAR_RATIO_GM3508,
    AgitatorSubsystem::RESERVOIR_42MM_MOTOR_ID,
    AgitatorSubsystem::RESERVOIR_42MM_MOTOR_CAN_BUS,
    AgitatorSubsystem::RESERVOIR_42MM_INVERTED);
EngineerWristSubsystem wrist(drivers());

/* define commands ----------------------------------------------------------*/
AgitatorCalibrateCommand reservoir17mmCalibrateCommand(&reservoir17mm);
AgitatorRotateCommand reservoir17mmRotateCommand(
    &reservoir17mm,
    2.0f * aruwlib::algorithms::PI / 3.0f,
    500,
    0,
    true);
AgitatorCalibrateCommand reservoir42mmCalibrateCommand(&reservoir42mm);
AgitatorRotateCommand reservoir42mmRotateCommand(
    &reservoir42mm,
    2.0f * aruwlib::algorithms::PI / 3.0f,
    500,
    0,
    true);
EngineerWristCalibrateCommand calibrateWrist(&wrist);
EngineerWristRotateCommand rotateWristForward(&wrist, EngineerWristSubsystem::MAX_WRIST_ANGLE);
EngineerWristRotateCommand rotateWristBack(&wrist, EngineerWristSubsystem::MIN_WRIST_ANGLE);

/* initialize subsystems ----------------------------------------------------*/
void initializeSubsystems() {}

/* register subsystems here -------------------------------------------------*/
void registerEngineerSubsystems(aruwlib::Drivers *drivers)
{
    drivers->commandScheduler.registerSubsystem(&grabber);
    drivers->commandScheduler.registerSubsystem(&xAxis);
    drivers->commandScheduler.registerSubsystem(&reservoir17mm);
    drivers->commandScheduler.registerSubsystem(&wrist);
}

/* set any default commands to subsystems here ------------------------------*/
void setDefaultEngineerCommands(aruwlib::Drivers *) {}

/* add any starting commands to the scheduler here --------------------------*/
void startEngineerCommands(aruwlib::Drivers *) {}

/* register io mappings here ------------------------------------------------*/
void registerEngineerIoMappings(aruwlib::Drivers *) {}

void initSubsystemCommands(aruwlib::Drivers *drivers)
{
    initializeSubsystems();
    registerEngineerSubsystems(drivers);
    setDefaultEngineerCommands(drivers);
    startEngineerCommands(drivers);
    registerEngineerIoMappings(drivers);
}

}  // namespace control

}  // namespace aruwsrc

#endif
