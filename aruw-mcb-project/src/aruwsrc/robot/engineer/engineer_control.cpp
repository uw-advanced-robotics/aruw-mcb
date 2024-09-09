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

#include "tap/control/command_scheduler.hpp"

#include "aruwsrc/robot/engineer/arm/joint_subsystem.hpp"
#include "aruwsrc/robot/engineer/arm/pitch_subsystem.hpp"
#include "aruwsrc/robot/engineer/arm/arm_superstructure.hpp"
#include "aruwsrc/robot/engineer/grabber/grabber_subsystem.hpp"


#include "aruwsrc/communication/sensors/current/acs712_current_sensor_config.hpp"
#include "aruwsrc/control/chassis/x_drive_chassis_subsystem.hpp"
#include "aruwsrc/control/safe_disconnect.hpp"
#include "aruwsrc/drivers_singleton.hpp"

#include "engineer_arm_constants.hpp"
#include "engineer_drivers.hpp"

using tap::control::CommandMapper;
using namespace aruwsrc::control;
using namespace aruwsrc::engineer;
using namespace aruwsrc::engineer::arm;
using namespace aruwsrc::engineer::grabber;
/*
 * NOTE: We are using the DoNotUse_getDrivers() function here
 *      because this file defines all subsystems and command
 *      and thus we must pass in the single statically allocated
 *      Drivers class to all of these objects.
 */
driversFunc drivers = DoNotUse_getDrivers;

namespace engineer_control
{
/* define subsystems --------------------------------------------------------*/
tap::communication::sensors::current::AnalogCurrentSensor currentSensor(
    {&drivers()->analog,
     aruwsrc::chassis::CURRENT_SENSOR_PIN,
     aruwsrc::communication::sensors::current::ACS712_CURRENT_SENSOR_MV_PER_MA,
     aruwsrc::communication::sensors::current::ACS712_CURRENT_SENSOR_ZERO_MA,
     aruwsrc::communication::sensors::current::ACS712_CURRENT_SENSOR_LOW_PASS_ALPHA});

aruwsrc::chassis::XDriveChassisSubsystem chassis(drivers(), &currentSensor);

tap::motor::DjiMotor temp(drivers(), tap::motor::MOTOR1, tap::can::CanBus::CAN_BUS1, false, "temp");

JointSubsystem xAxis(drivers(), xAxisConfig, &temp, "X axis joint");
JointSubsystem lift(drivers(), LiftConfig, nullptr, "Lift joint");
PitchSubsystem pitch(drivers(), pitchConfig, nullptr);
JointSubsystem yaw(drivers(), yawConfig, nullptr, "Yaw joint");
JointSubsystem roll(drivers(), rollConfig, nullptr, "Roll joint");

ArmSuperstructure superstructure(&xAxis, &lift, &yaw, &pitch, &roll);

GrabberSubsystem grabber(drivers(), nullptr);

/* define commands ----------------------------------------------------------*/

// Safe disconnect function
RemoteSafeDisconnectFunction remoteSafeDisconnectFunction(drivers());

/* initialize subsystems ----------------------------------------------------*/
void initializeSubsystems()
{
    chassis.initialize();
    xAxis.initialize();
    lift.initialize();
    pitch.initialize();
    yaw.initialize();
    roll.initialize();
}

/* register subsystems here -------------------------------------------------*/
void registerEngineerSubsystems(aruwsrc::engineer::Drivers *) {}

/* set any default commands to subsystems here ------------------------------*/
void setDefaultEngineerCommands(aruwsrc::engineer::Drivers *) {}

/* add any starting commands to the scheduler here --------------------------*/
void startEngineerCommands(aruwsrc::engineer::Drivers *) {}

/* register io mappings here ------------------------------------------------*/
void registerEngineerIoMappings(aruwsrc::engineer::Drivers *) {}

}  // namespace engineer_control

namespace aruwsrc::engineer
{
void initSubsystemCommands(aruwsrc::engineer::Drivers *drivers)
{
    drivers->commandScheduler.setSafeDisconnectFunction(
        &engineer_control::remoteSafeDisconnectFunction);
    engineer_control::initializeSubsystems();
    engineer_control::registerEngineerSubsystems(drivers);
    engineer_control::setDefaultEngineerCommands(drivers);
    engineer_control::startEngineerCommands(drivers);
    engineer_control::registerEngineerIoMappings(drivers);
}
}  // namespace aruwsrc::engineer

#endif
