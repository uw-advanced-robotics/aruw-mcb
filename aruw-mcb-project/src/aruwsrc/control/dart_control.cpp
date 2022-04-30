#ifdef TARGET_DART
#include "tap/algorithms/math_user_utils.hpp"
#include "tap/control/command_mapper.hpp"
#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/hold_repeat_command_mapping.hpp"
#include "tap/control/setpoint/commands/calibrate_command.hpp"
#include "tap/control/setpoint/commands/move_absolute_command.hpp"
#include "tap/motor/double_dji_motor.hpp"
#include "tap/motor/motor_interface.hpp"
#include "tap/motor/stepper_motor.hpp"
#include "turret/user/stepper_motor_turret_control_command.hpp"

#include "agitator/agitator_subsystem.hpp"
#include "agitator/move_unjam_ref_limited_command.hpp"
#include "aruwsrc/control/agitator/constants/agitator_constants.hpp"
#include "aruwsrc/control/safe_disconnect.hpp"
#include "aruwsrc/drivers.hpp"
#include "aruwsrc/drivers_singleton.hpp"
#include "aruwsrc/util_macros.hpp"
#include "launcher/friction_wheel_spin_ref_limited_command.hpp"
#include "launcher/friction_wheel_subsystem.hpp"
#include "turret/turret_subsystem.hpp"

using namespace tap::control::setpoint;
using namespace aruwsrc::control::turret;
using namespace aruwsrc::agitator;
using namespace aruwsrc::control::launcher;
using namespace aruwsrc::control;
using namespace tap::motor;
using namespace tap::control;
using tap::communication::serial::Remote;
using tap::control::setpoint::MoveAbsoluteCommand;

/*
 * NOTE: We are using the DoNotUse_getDrivers() function here
 *      because this file defines all subsystems and command
 *      and thus we must pass in the single statically allocated
 *      Drivers class to all of these objects.
 */
aruwsrc::driversFunc drivers = aruwsrc::DoNotUse_getDrivers;

namespace dart_control
{
/* define subsystems ----------------------------------------------------------*/
tap::motor::StepperMotor pitchMotor(drivers(), false, "pitchMotor", tap::gpio::Digital::OutputPin::E, tap::gpio::Pwm::Pin::X);
tap::motor::StepperMotor yawMotor(drivers(), false, "yawMotor", tap::gpio::Digital::OutputPin::F, tap::gpio::Pwm::Pin::W);

TurretSubsystem turret(drivers(), &pitchMotor, &yawMotor, false);

// Agitators
AgitatorSubsystem agitatorTop(
    drivers(),
    aruwsrc::control::agitator::constants::PID_AGITATOR_FEEDER,
    AgitatorSubsystem::AGITATOR_GEAR_RATIO_M2006,
    tap::motor::MOTOR5,
    tap::can::CanBus::CAN_BUS1,
    aruwsrc::control::agitator::constants::IS_AGITATOR_INVERTED,
    false,
    0,
    0);

AgitatorSubsystem agitatorBottom(
    drivers(),
    aruwsrc::control::agitator::constants::PID_AGITATOR_FEEDER,
    AgitatorSubsystem::AGITATOR_GEAR_RATIO_M2006,
    tap::motor::MOTOR5,
    tap::can::CanBus::CAN_BUS2,
    aruwsrc::control::agitator::constants::IS_AGITATOR_INVERTED,
    false,
    0,
    0);

// Friction Wheels
FrictionWheelSubsystem frictionWheelsTopFront(
    drivers(),
    tap::motor::MOTOR1,
    tap::motor::MOTOR2,
    tap::can::CanBus::CAN_BUS1);

FrictionWheelSubsystem frictionWheelsTopBack(
    drivers(),
    tap::motor::MOTOR3,
    tap::motor::MOTOR4,
    tap::can::CanBus::CAN_BUS1);

FrictionWheelSubsystem frictionWheelsBottomFront(
    drivers(),
    tap::motor::MOTOR1,
    tap::motor::MOTOR2,
    tap::can::CanBus::CAN_BUS2);

FrictionWheelSubsystem frictionWheelsBottomBack(
    drivers(),
    tap::motor::MOTOR3,
    tap::motor::MOTOR4,
    tap::can::CanBus::CAN_BUS2);



/* define commands ----------------------------------------------------------*/
// Agitator Commands
static constexpr float AGITATOR_TARGET_ANGLE_ZERO = 0;

static constexpr float AGITATOR_TARGET_ANGLE_ONE = 8 * M_PI;

static constexpr float AGITATOR_TARGET_ANGLE_TWO = 17.5 * M_PI;

static constexpr uint32_t AGITATOR_ANGULAR_SPEED = 5.0f * M_PI * 1000.0f;

static constexpr float AGITATOR_TOLERANCE = 0.05f;

CalibrateCommand agitatorTopCalibrateCommand(&agitatorTop);

CalibrateCommand agitatorBottomCalibrateCommand(&agitatorBottom);

MoveAbsoluteCommand agitatorTopMoveCommandZero(
    &agitatorTop,
    AGITATOR_TARGET_ANGLE_ZERO,
    AGITATOR_ANGULAR_SPEED,
    AGITATOR_TOLERANCE,
    true,
    true);

MoveAbsoluteCommand agitatorTopMoveCommandOne(
    &agitatorTop,
    AGITATOR_TARGET_ANGLE_ONE,
    AGITATOR_ANGULAR_SPEED,
    AGITATOR_TOLERANCE,
    true,
    true);

MoveAbsoluteCommand agitatorTopMoveCommandTwo(
    &agitatorTop,
    AGITATOR_TARGET_ANGLE_TWO,
    AGITATOR_ANGULAR_SPEED,
    AGITATOR_TOLERANCE,
    true,
    true);

MoveAbsoluteCommand agitatorBottomMoveCommandZero(
    &agitatorBottom,
    AGITATOR_TARGET_ANGLE_ZERO,
    AGITATOR_ANGULAR_SPEED,
    AGITATOR_TOLERANCE,
    true,
    true);

MoveAbsoluteCommand agitatorBottomMoveCommandOne(
    &agitatorBottom,
    AGITATOR_TARGET_ANGLE_ONE,
    AGITATOR_ANGULAR_SPEED,
    AGITATOR_TOLERANCE,
    true,
    true);

MoveAbsoluteCommand agitatorBottomMoveCommandTwo(
    &agitatorBottom,
    AGITATOR_TARGET_ANGLE_TWO,
    AGITATOR_ANGULAR_SPEED,
    AGITATOR_TOLERANCE,
    true,
    true);

// Starting Friction Wheels
FrictionWheelSpinRefLimitedCommand spinFrictionWheelsTopBack(
    drivers(),
    &frictionWheelsTopBack,
    15.0f,
    false,
    FrictionWheelSpinRefLimitedCommand::Barrel::BARREL_17MM_1);

FrictionWheelSpinRefLimitedCommand spinFrictionWheelsTopFront(
    drivers(),
    &frictionWheelsTopFront,
    15.0f,
    false,
    FrictionWheelSpinRefLimitedCommand::Barrel::BARREL_17MM_1);

FrictionWheelSpinRefLimitedCommand spinFrictionWheelsBottomFront(
    drivers(),
    &frictionWheelsBottomFront,
    15.0f,
    false,
    FrictionWheelSpinRefLimitedCommand::Barrel::BARREL_17MM_1);

FrictionWheelSpinRefLimitedCommand spinFrictionWheelsBottomBack(
    drivers(),
    &frictionWheelsBottomBack,
    15.0f,
    false,
    FrictionWheelSpinRefLimitedCommand::Barrel::BARREL_17MM_1);

// Stopping Friction Wheels
FrictionWheelSpinRefLimitedCommand stopFrictionWheelsTopFront(
    drivers(),
    &frictionWheelsTopFront,
    0.0f,
    true,
    FrictionWheelSpinRefLimitedCommand::Barrel::BARREL_17MM_1);

FrictionWheelSpinRefLimitedCommand stopFrictionWheelsTopBack(
    drivers(),
    &frictionWheelsTopBack,
    0.0f,
    true,
    FrictionWheelSpinRefLimitedCommand::Barrel::BARREL_17MM_1);

FrictionWheelSpinRefLimitedCommand stopFrictionWheelsBottomFront(
    drivers(),
    &frictionWheelsBottomFront,
    0.0f,
    true,
    FrictionWheelSpinRefLimitedCommand::Barrel::BARREL_17MM_1);

FrictionWheelSpinRefLimitedCommand stopFrictionWheelsBottomBack(
    drivers(),
    &frictionWheelsBottomBack,
    0.0f,
    true,
    FrictionWheelSpinRefLimitedCommand::Barrel::BARREL_17MM_1);

aruwsrc::control::turret::user::StepperMotorTurretControlCommand stepperMotorTurretControlCommand(
    drivers(),
    &turret
);

// Mappings
HoldCommandMapping bottomPositionOne(
    drivers(),
    {&agitatorBottomMoveCommandOne},
    RemoteMapState(Remote::SwitchState::MID, Remote::SwitchState::MID));

HoldCommandMapping bottomPositionTwo(
    drivers(),
    {&agitatorBottomMoveCommandTwo},
    RemoteMapState(Remote::SwitchState::UP, Remote::SwitchState::MID));

HoldCommandMapping topPositionTwo(
    drivers(),
    {&agitatorTopMoveCommandTwo},
    RemoteMapState(Remote::SwitchState::UP, Remote::SwitchState::UP));

HoldCommandMapping topPositionOne(
    drivers(),
    {&agitatorTopMoveCommandOne},
    RemoteMapState(Remote::SwitchState::MID, Remote::SwitchState::UP));

HoldCommandMapping positionZero(
    drivers(),
    {&agitatorTopMoveCommandZero, &agitatorBottomMoveCommandZero},
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN));

HoldCommandMapping stopWheels(
    drivers(),
    {&stopFrictionWheelsBottomBack,
     &stopFrictionWheelsBottomFront,
     &stopFrictionWheelsTopBack,
     &stopFrictionWheelsTopFront},
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::DOWN));

// Safe disconnect function
RemoteSafeDisconnectFunction remoteSafeDisconnectFunction(drivers());

void registerDartSubsystems(aruwsrc::Drivers *drivers)
{
    drivers->commandScheduler.registerSubsystem(&agitatorTop);
    drivers->commandScheduler.registerSubsystem(&agitatorBottom);
    drivers->commandScheduler.registerSubsystem(&frictionWheelsTopFront);
    drivers->commandScheduler.registerSubsystem(&frictionWheelsTopBack);
    drivers->commandScheduler.registerSubsystem(&frictionWheelsBottomFront);
    drivers->commandScheduler.registerSubsystem(&frictionWheelsBottomBack);
}

void initializeSubsystems()
{
    agitatorBottom.initialize();
    agitatorTop.initialize();
    frictionWheelsBottomBack.initialize();
    frictionWheelsBottomFront.initialize();
    frictionWheelsTopBack.initialize();
    frictionWheelsTopFront.initialize();
    turret.initialize();
}

void startDartCommands(aruwsrc::Drivers *drivers)
{
    drivers->commandScheduler.addCommand(&agitatorTopCalibrateCommand);
    drivers->commandScheduler.addCommand(&agitatorBottomCalibrateCommand);
}

void setDefaultDartCommands(aruwsrc::Drivers *)
{
    frictionWheelsTopFront.setDefaultCommand(&spinFrictionWheelsTopFront);
    frictionWheelsTopBack.setDefaultCommand(&spinFrictionWheelsTopBack);
    frictionWheelsBottomFront.setDefaultCommand(&spinFrictionWheelsBottomFront);
    frictionWheelsBottomBack.setDefaultCommand(&spinFrictionWheelsBottomBack);
    turret.setDefaultCommand(&stepperMotorTurretControlCommand);
}

/* register io mappings here ------------------------------------------------*/
void registerDartIoMappings(aruwsrc::Drivers *drivers)
{
    drivers->commandMapper.addMap(&topPositionOne);
    drivers->commandMapper.addMap(&topPositionTwo);
    drivers->commandMapper.addMap(&bottomPositionOne);
    drivers->commandMapper.addMap(&bottomPositionTwo);
    drivers->commandMapper.addMap(&positionZero);
    drivers->commandMapper.addMap(&stopWheels);
}
}  // namespace dart_control
namespace aruwsrc::control
{
void initSubsystemCommands(aruwsrc::Drivers *drivers)
{
    drivers->commandScheduler.setSafeDisconnectFunction(
        &dart_control::remoteSafeDisconnectFunction);
    dart_control::initializeSubsystems();
    dart_control::registerDartSubsystems(drivers);
    dart_control::startDartCommands(drivers);
    dart_control::registerDartIoMappings(drivers);
    dart_control::setDefaultDartCommands(drivers);
}
}  // namespace aruwsrc::control

#endif
