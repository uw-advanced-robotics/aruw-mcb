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

#include "aruwsrc/util_macros.hpp"

#ifdef TARGET_BALSTD

#include "tap/communication/sensors/current/analog_current_sensor.hpp"
#include "tap/control/command_mapper.hpp"
#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/hold_repeat_command_mapping.hpp"
#include "tap/control/press_command_mapping.hpp"
#include "tap/control/setpoint/commands/calibrate_command.hpp"
#include "tap/control/toggle_command_mapping.hpp"
#include "tap/drivers.hpp"

#include "aruwsrc/algorithms/odometry/otto_kf_odometry_2d_subsystem.hpp"
#include "aruwsrc/algorithms/odometry/standard_and_hero_transform_adapter.hpp"
#include "aruwsrc/algorithms/odometry/standard_and_hero_transformer.hpp"
#include "aruwsrc/algorithms/odometry/standard_and_hero_transformer_subsystem.hpp"
#include "aruwsrc/communication/sensors/current/acs712_current_sensor_config.hpp"
#include "aruwsrc/control/buzzer/buzzer_subsystem.hpp"
#include "aruwsrc/control/imu/imu_calibrate_command.hpp"
#include "aruwsrc/control/safe_disconnect.hpp"
#include "aruwsrc/display/imu_calibrate_menu.hpp"
#include "aruwsrc/drivers_singleton.hpp"
#include "aruwsrc/robot/balstd/balstd_chassis_subsystem.hpp"
#include "aruwsrc/robot/balstd/balstd_drivers.hpp"

#ifdef PLATFORM_HOSTED
#include "tap/communication/can/can.hpp"
#endif

using namespace aruwsrc::balstd;
using namespace aruwsrc::control;
using namespace aruwsrc::control::balstd;
using namespace aruwsrc::control::turret;
using namespace aruwsrc::algorithms;
using namespace aruwsrc::algorithms::odometry;
using namespace aruwsrc::algorithms::transforms;
using namespace tap::control::setpoint;
using namespace tap::control;
using namespace tap::communication::serial;

/*
 * NOTE: We are using the DoNotUse_getDrivers() function here
 *      because this file defines all subsystems and command
 *      and thus we must pass in the single statically allocated
 *      Drivers class to all of these objects.
 */
driversFunc drivers = DoNotUse_getDrivers;

namespace balstd_control
{
inline aruwsrc::can::TurretMCBCanComm &getTurretMCBCanComm()
{
    return drivers()->turretMCBCanCommBus1;
}

/* define subsystems --------------------------------------------------------*/

tap::communication::sensors::current::AnalogCurrentSensor currentSensor(
    {&drivers()->analog,
     aruwsrc::chassis::CURRENT_SENSOR_PIN,
     aruwsrc::communication::sensors::current::ACS712_CURRENT_SENSOR_MV_PER_MA,
     aruwsrc::communication::sensors::current::ACS712_CURRENT_SENSOR_ZERO_MA,
     aruwsrc::communication::sensors::current::ACS712_CURRENT_SENSOR_LOW_PASS_ALPHA});

aruwsrc::chassis::BalstdChassisSubsystem chassis(
    drivers(),
    &currentSensor,
    &drivers()->capacitorBank);

OttoKFOdometry2DSubsystem odometrySubsystem(*drivers(), turret, chassis, modm::Vector2f(0, 0));

// transforms
// StandardAndHeroTransformer transformer(odometrySubsystem, turret);
// StandardAnderHeroTransformerSubsystem transformSubsystem(*drivers(), transformer);

// StandardAndHeroTransformAdapter transformAdapter(transformer);

/* define commands ----------------------------------------------------------*/

// imu::ImuCalibrateCommand imuCalibrateCommand(
//     drivers(),
//     {{
//         &getTurretMCBCanComm(),
//         &turret,
//         &chassisFrameYawTurretController,
//         &chassisFramePitchTurretController,
//         true,
//     }},
//     &chassis);

aruwsrc::control::buzzer::BuzzerSubsystem buzzer(drivers());

/* define command mappings --------------------------------------------------*/

// Remote related mappings
HoldCommandMapping rightSwitchDown(
    drivers(),
    {&stopFrictionWheels},
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::DOWN));
HoldRepeatCommandMapping rightSwitchUp(
    drivers(),
    {&rotateAndUnjamAgitatorWithHeatAndCVLimiting},
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP),
    true);
HoldCommandMapping leftSwitchDown(
    drivers(),
    {&beybladeCommand},
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN));
HoldCommandMapping leftSwitchUp(
    drivers(),
    {&turretCVCommand, &chassisDriveCommand},
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP));

CycleStateCommandMapping<bool, 2, CvOnTargetGovernor> rPressed(
    drivers(),
    RemoteMapState({Remote::Key::R}),
    true,
    &cvOnTargetGovernor,
    &CvOnTargetGovernor::setGovernorEnabled);

ToggleCommandMapping fToggled(drivers(), {&beybladeCommand}, RemoteMapState({Remote::Key::F}));

MultiShotCvCommandMapping leftMousePressedBNotPressed(
    *drivers(),
    rotateAndUnjamAgitatorWithHeatAndCVLimiting,
    RemoteMapState(RemoteMapState::MouseButton::LEFT, {}, {Remote::Key::B}),
    &manualFireRateReselectionManager,
    cvOnTargetGovernor,
    &rotateAgitator);

HoldRepeatCommandMapping leftMousePressedBPressed(
    drivers(),
    {&rotateAndUnjamAgitatorWhenFrictionWheelsOnUntilProjectileLaunched},
    RemoteMapState(RemoteMapState::MouseButton::LEFT, {Remote::Key::B}),
    false);
HoldCommandMapping rightMousePressed(
    drivers(),
    {&turretCVCommand},
    RemoteMapState(RemoteMapState::MouseButton::RIGHT));
PressCommandMapping zPressed(drivers(), {&turretUTurnCommand}, RemoteMapState({Remote::Key::Z}));
// The "right switch down" portion is to avoid accidentally recalibrating in the middle of a match.
PressCommandMapping bNotCtrlPressedRightSwitchDown(
    drivers(),
    {&imuCalibrateCommand},
    RemoteMapState(
        Remote::SwitchState::UNKNOWN,
        Remote::SwitchState::DOWN,
        {Remote::Key::B},
        {Remote::Key::CTRL},
        false,
        false));
// The user can press b+ctrl when the remote right switch is in the down position to restart the
// client display command. This is necessary since we don't know when the robot is connected to the
// server and thus don't know when to start sending the initial HUD graphics.
PressCommandMapping bCtrlPressed(
    drivers(),
    {&clientDisplayCommand},
    RemoteMapState({Remote::Key::CTRL, Remote::Key::B}));

// The user can press q and e simultaneously to enable wiggle driving. Wiggling is cancelled
// automatically once a different drive mode is chosen.
ToggleCommandMapping qPressed(drivers(), {&wiggleCommand}, RemoteMapState({Remote::Key::Q}));

PressCommandMapping xPressed(
    drivers(),
    {&chassisAutorotateCommand},
    RemoteMapState({Remote::Key::X}));

CycleStateCommandMapping<
    MultiShotCvCommandMapping::LaunchMode,
    MultiShotCvCommandMapping::NUM_SHOOTER_STATES,
    MultiShotCvCommandMapping>
    vPressed(
        drivers(),
        RemoteMapState({Remote::Key::V}),
        MultiShotCvCommandMapping::SINGLE,
        &leftMousePressedBNotPressed,
        &MultiShotCvCommandMapping::setShooterState);

// cap bank
PressCommandMapping cShiftPressed(
    drivers(),
    {&capBankToggleCommand},
    RemoteMapState({Remote::Key::SHIFT, Remote::Key::C}));
HoldCommandMapping shiftPressed(
    drivers(),
    {&capBankSprintCommand},
    RemoteMapState({Remote::Key::SHIFT}));
HoldCommandMapping ctrlPressed(
    drivers(),
    {&capBankHalfSprintCommand},
    RemoteMapState({Remote::Key::CTRL}));

// Safe disconnect function
RemoteSafeDisconnectFunction remoteSafeDisconnectFunction(drivers());

/* register subsystems here -------------------------------------------------*/
void registerStandardSubsystems(Drivers *drivers)
{
    drivers->commandScheduler.registerSubsystem(&agitator);
    drivers->commandScheduler.registerSubsystem(&chassis);
    drivers->commandScheduler.registerSubsystem(&odometrySubsystem);
    drivers->commandScheduler.registerSubsystem(&buzzer);
    drivers->commandScheduler.registerSubsystem(&transformSubsystem);
}

/* initialize subsystems ----------------------------------------------------*/
void initializeSubsystems()
{
    chassis.initialize();
    odometrySubsystem.initialize();
    buzzer.initialize();
    transformSubsystem.initialize();
}

/* set any default commands to subsystems here ------------------------------*/
void setDefaultStandardCommands(Drivers *) { chassis.setDefaultCommand(&chassisAutorotateCommand); }

/* add any starting commands to the scheduler here --------------------------*/
void startStandardCommands(Drivers *drivers)
{
    // drivers->commandScheduler.addCommand(&clientDisplayCommand);
    drivers->commandScheduler.addCommand(&imuCalibrateCommand);
    drivers->visionCoprocessor.attachTransformer(&transformAdapter);
}

/* register io mappings here ------------------------------------------------*/
void registerStandardIoMappings(Drivers *drivers)
{
    // drivers->commandMapper.addMap(&rightSwitchDown);
    // drivers->commandMapper.addMap(&rightSwitchUp);
    // drivers->commandMapper.addMap(&leftSwitchDown);
    // drivers->commandMapper.addMap(&leftSwitchUp);
    // drivers->commandMapper.addMap(&rPressed);
    // drivers->commandMapper.addMap(&fToggled);
    // drivers->commandMapper.addMap(&leftMousePressedBNotPressed);
    // drivers->commandMapper.addMap(&leftMousePressedBPressed);
    // drivers->commandMapper.addMap(&rightMousePressed);
    // drivers->commandMapper.addMap(&zPressed);
    // drivers->commandMapper.addMap(&bNotCtrlPressedRightSwitchDown);
    // drivers->commandMapper.addMap(&bCtrlPressed);
    // drivers->commandMapper.addMap(&qPressed);
    // drivers->commandMapper.addMap(&xPressed);
    // drivers->commandMapper.addMap(&vPressed);
    // drivers->commandMapper.addMap(&cShiftPressed);
    // drivers->commandMapper.addMap(&shiftPressed);
    // drivers->commandMapper.addMap(&ctrlPressed);
}
}  // namespace balstd_control

namespace aruwsrc::balstd
{
void initSubsystemCommands(aruwsrc::standard::Drivers *drivers)
{
    drivers->commandScheduler.setSafeDisconnectFunction(
        &balstd_control::remoteSafeDisconnectFunction);
    balstd_control::initializeSubsystems();
    balstd_control::registerStandardSubsystems(drivers);
    balstd_control::setDefaultStandardCommands(drivers);
    balstd_control::startStandardCommands(drivers);
    balstd_control::registerStandardIoMappings(drivers);
}
}  // namespace aruwsrc::balstd

#ifndef PLATFORM_HOSTED
imu::ImuCalibrateCommand *getImuCalibrateCommand()
{
    return &standard_control::imuCalibrateCommand;
}
#endif

#endif
