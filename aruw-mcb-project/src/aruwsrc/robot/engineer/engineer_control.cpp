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

#if defined(TARGET_ENGINEER)

#include "tap/communication/gpio/digital.hpp"
#include "tap/control/command_mapper.hpp"
#include "tap/control/command_scheduler.hpp"
#include "tap/control/hold_command_mapping.hpp"

#include "aruwsrc/control/safe_disconnect.hpp"
#include "aruwsrc/drivers_singleton.hpp"
#include "aruwsrc/robot/engineer/cube_move_command.hpp"
#include "aruwsrc/robot/engineer/cube_storage_subsystem.hpp"
#include "aruwsrc/robot/engineer/engineer_drivers.hpp"
using namespace tap::gpio;
using tap::communication::serial::Remote;
using tap::control::CommandMapper;
using namespace aruwsrc::engineer;
using namespace aruwsrc::robot::engineer;
using namespace tap::control;

/*
 * NOTE: We are using the DoNotUse_getDrivers() function here
 *      because this file defines all subsystems and command
 *      and thus we must pass in the single statically allocated
 *      Drivers class to all of these objects.
 */
driversFunc drivers = DoNotUse_getDrivers;

namespace aruwsrc
{
namespace control
{
static constexpr Digital::OutputPin GRABBER_PIN = Digital::OutputPin::E;
static constexpr Digital::OutputPin X_AXIS_PIN = Digital::OutputPin::F;
static constexpr Digital::OutputPin TOWER_LEFT_PIN = Digital::OutputPin::G;
static constexpr Digital::OutputPin TOWER_RIGHT_PIN = Digital::OutputPin::H;
static constexpr Digital::InputPin TOWER_LEFT_LIMIT_SWITCH = Digital::InputPin::B;
static constexpr Digital::InputPin TOWER_RIGHT_LIMIT_SWITCH = Digital::InputPin::C;

tap::motor::DjiMotor storageLiftMotor(
    drivers(),
    CUBE_LIFT_MOTOR_ID,
    LIFT_MOTOR_CAN_BUS,
    false,
    "Lifting Motor");

/* define subsystems --------------------------------------------------------*/

CubeStorageSubsystem cubeLift(drivers(), storageLiftMotor);
/* define commands ----------------------------------------------------------*/
CubeMoveCommand cubeUp(cubeLift, 1000);
CubeMoveCommand cubeDown(cubeLift, -1000);
// Safe disconnect function
RemoteSafeDisconnectFunction remoteSafeDisconnectFunction(drivers());

tap::control::HoldCommandMapping rightSwitchUp(
    drivers(),
    {&cubeUp},
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP));

tap::control::HoldCommandMapping rightSwitchDown(
    drivers(),
    {&cubeDown},
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::DOWN));

/* initialize subsystems ----------------------------------------------------*/
void initializeSubsystems() { cubeLift.initialize(); }

/* register subsystems here -------------------------------------------------*/
void registerEngineerSubsystems(aruwsrc::engineer::Drivers *drivers)
{
    drivers->commandScheduler.registerSubsystem(&cubeLift);
}

/* set any default commands to subsystems here ------------------------------*/
void setDefaultEngineerCommands(aruwsrc::engineer::Drivers *) {}

/* add any starting commands to the scheduler here --------------------------*/
void startEngineerCommands(aruwsrc::engineer::Drivers *) {}

/* register io mappings here ------------------------------------------------*/
void registerEngineerIoMappings(aruwsrc::engineer::Drivers *drivers)
{
    drivers->commandMapper.addMap(&rightSwitchUp);
    drivers->commandMapper.addMap(&rightSwitchDown);
}
}  // namespace control

}  // namespace aruwsrc

namespace aruwsrc::engineer
{
void initSubsystemCommands(aruwsrc::engineer::Drivers *drivers)
{
    drivers->commandScheduler.setSafeDisconnectFunction(
        &aruwsrc::control::remoteSafeDisconnectFunction);
    aruwsrc::control::initializeSubsystems();
    aruwsrc::control::registerEngineerSubsystems(drivers);
    aruwsrc::control::setDefaultEngineerCommands(drivers);
    aruwsrc::control::startEngineerCommands(drivers);
    aruwsrc::control::registerEngineerIoMappings(drivers);
}
}  // namespace aruwsrc::engineer

#endif
