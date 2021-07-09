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

#include "agitator_subsystem_mock.hpp"
#include "beyblade_command_mock.hpp"
#include "chassis_drive_command_mock.hpp"
#include "chassis_subsystem_mock.hpp"
#include "friction_wheel_subsystem_mock.hpp"
#include "grabber_subsystem_mock.hpp"
#include "hopper_subsystem_mock.hpp"
#include "oled_display_mock.hpp"
#include "sentinel_drive_subsystem_mock.hpp"
#include "sentinel_switcher_subsystem_mock.hpp"
#include "tow_subsystem_mock.hpp"
#include "turret_subsystem_mock.hpp"
#include "x_axis_subsystem_mock.hpp"
#include "xavier_serial_mock.hpp"

// A file for listing all mock constructors and destructors since doing
// so in a source file allows for faster compilation than defining constructors
// in the headers
namespace aruwsrc::mock
{
AgitatorSubsystemMock::AgitatorSubsystemMock(
    aruwlib::Drivers *drivers,
    const aruwlib::algorithms::PidConfigStruct &pidConfig,
    float agitatorGearRatio,
    aruwlib::motor::MotorId agitatorMotorId,
    aruwlib::can::CanBus agitatorCanBusId,
    bool isAgitatorInverted,
    bool jamLogicEnabled,
    float jamDistanceTolerance,
    uint32_t jamTemporalTolerance)
    : AgitatorSubsystem(
          drivers,
          pidConfig,
          agitatorGearRatio,
          agitatorMotorId,
          agitatorCanBusId,
          isAgitatorInverted,
          jamLogicEnabled,
          jamDistanceTolerance,
          jamTemporalTolerance)
{
    ON_CALL(*this, isOnline).WillByDefault(testing::Return(true));
}
AgitatorSubsystemMock::~AgitatorSubsystemMock() {}

BeybladeCommandMock::BeybladeCommandMock(
    aruwlib::Drivers *drivers,
    chassis::ChassisSubsystem *chassis,
    aruwlib::control::turret::TurretSubsystemInterface *turret)
    : BeybladeCommand(drivers, chassis, turret)
{
}
BeybladeCommandMock::~BeybladeCommandMock() {}

ChassisDriveCommandMock::ChassisDriveCommandMock(aruwlib::Drivers *d, chassis::ChassisSubsystem *cs)
    : chassis::ChassisDriveCommand(d, cs)
{
}
ChassisDriveCommandMock::~ChassisDriveCommandMock() {}

ChassisSubsystemMock::ChassisSubsystemMock(
    aruwlib::Drivers *drivers,
    const ChassisMechanicalConstants &mechanicalConstants,
    const aruwlib::algorithms::PidConfigStruct &velocityPidConfig,
    const aruwlib::control::chassis::PowerLimiterConfig &powerLimiterConfig,
    float maxWheelSpeedSingleMotor,
    float chassisRevolvePidMaxP,
    float chassisRevolvePidMaxD,
    float chassisRevolvePidKD,
    float chassisRevolvePidMaxOutput,
    float minErrorRotationD,
    float minRotationThreshold,
    aruwlib::can::CanBus canBus,
    aruwlib::motor::MotorId leftFrontMotorId,
    aruwlib::motor::MotorId leftBackMotorId,
    aruwlib::motor::MotorId rightFrontMotorId,
    aruwlib::motor::MotorId rightBackMotorId,
    aruwlib::gpio::Analog::Pin currentPin)
    : ChassisSubsystem(
          drivers,
          mechanicalConstants,
          velocityPidConfig,
          powerLimiterConfig,
          maxWheelSpeedSingleMotor,
          chassisRevolvePidMaxP,
          chassisRevolvePidMaxD,
          chassisRevolvePidKD,
          chassisRevolvePidMaxOutput,
          minErrorRotationD,
          minRotationThreshold,
          canBus,
          leftFrontMotorId,
          leftBackMotorId,
          rightFrontMotorId,
          rightBackMotorId,
          currentPin)
{
}
ChassisSubsystemMock::~ChassisSubsystemMock() {}

FrictionWheelSubsystemMock::FrictionWheelSubsystemMock(
    aruwlib::Drivers *drivers,
    const aruwlib::algorithms::PidConfigStruct &velocityPidConfig,
    float frictionWheelRampSpeed,
    aruwlib::motor::MotorId leftMotor,
    aruwlib::motor::MotorId rightMotor,
    aruwlib::can::CanBus canBus)
    : FrictionWheelSubsystem(
          drivers,
          velocityPidConfig,
          frictionWheelRampSpeed,
          leftMotor,
          rightMotor,
          canBus)
{
}
FrictionWheelSubsystemMock::~FrictionWheelSubsystemMock() {}

GrabberSubsystemMock::GrabberSubsystemMock(
    aruwlib::Drivers *drivers,
    aruwlib::gpio::Digital::OutputPin pin)
    : engineer::GrabberSubsystem(drivers, pin)
{
}
GrabberSubsystemMock::~GrabberSubsystemMock() {}

OledDisplayMock::OledDisplayMock(aruwlib::Drivers *drivers) : display::OledDisplay(drivers) {}
OledDisplayMock::~OledDisplayMock() {}

HopperSubsystemMock::HopperSubsystemMock(
    aruwlib::Drivers *drivers,
    aruwlib::gpio::Pwm::Pin pwmPin,
    float open,
    float close,
    float pwmRampSpeed)
    : control::HopperSubsystem(drivers, pwmPin, open, close, pwmRampSpeed)
{
}
HopperSubsystemMock::~HopperSubsystemMock() {}

SentinelDriveSubsystemMock::SentinelDriveSubsystemMock(

    aruwlib::Drivers *drivers,
    const aruwlib::algorithms::PidConfigStruct &velocityPidConfig,
    const aruwlib::control::chassis::PowerLimiterConfig &powerLimiterConfig,
    aruwlib::gpio::Digital::InputPin leftLimitSwitch,
    aruwlib::gpio::Digital::InputPin rightLimitSwitch,
    aruwlib::gpio::Analog::Pin currentSensorPin,
    float wheelRadius,
    float gearRatio,
    float railLength,
    float sentinelLength,
    aruwlib::motor::MotorId leftMotorId,
    aruwlib::motor::MotorId rightMotorId,
    aruwlib::can::CanBus chassisCanBus)
    : control::sentinel::drive::SentinelDriveSubsystem(
          drivers,
          velocityPidConfig,
          powerLimiterConfig,
          leftLimitSwitch,
          rightLimitSwitch,
          currentSensorPin,
          wheelRadius,
          gearRatio,
          railLength,
          sentinelLength,
          leftMotorId,
          rightMotorId,
          chassisCanBus)
{
}
SentinelDriveSubsystemMock::~SentinelDriveSubsystemMock() {}

SentinelSwitcherSubsystemMock::SentinelSwitcherSubsystemMock(
    aruwlib::Drivers *drivers,
    aruwlib::gpio::Pwm::Pin switcherServoPin)
    : control::sentinel::firing::SentinelSwitcherSubsystem(drivers, switcherServoPin, 0, 0)
{
}
SentinelSwitcherSubsystemMock::~SentinelSwitcherSubsystemMock() {}

TowSubsystemMock::TowSubsystemMock(
    aruwlib::Drivers *drivers,
    aruwlib::gpio::Digital::OutputPin leftTowPin,
    aruwlib::gpio::Digital::OutputPin rightTowPin,
    aruwlib::gpio::Digital::InputPin leftTowLimitSwitchPin,
    aruwlib::gpio::Digital::InputPin rightTowLimitSwitchPin)
    : aruwsrc::engineer::TowSubsystem(
          drivers,
          leftTowPin,
          rightTowPin,
          leftTowLimitSwitchPin,
          rightTowLimitSwitchPin)
{
}
TowSubsystemMock::~TowSubsystemMock() {}

TurretSubsystemMock::TurretSubsystemMock(
    aruwlib::Drivers *drivers,
    float startAngle,
    float yawMinAngle,
    float yawMaxAngle,
    float pitchMinAngle,
    float pitchMaxAngle,
    float yawStartEncoderPosition,
    float pitchStartEncoderPosition,
    float feedForwardKp,
    float feedForwardMaxOutput,
    aruwlib::can::CanBus motorCanBus,
    aruwlib::motor::MotorId pitchMotorId,
    aruwlib::motor::MotorId yawMotorId)
    : TurretSubsystem(
          drivers,
          startAngle,
          yawMinAngle,
          yawMaxAngle,
          pitchMinAngle,
          pitchMaxAngle,
          yawStartEncoderPosition,
          pitchStartEncoderPosition,
          feedForwardKp,
          feedForwardMaxOutput,
          motorCanBus,
          pitchMotorId,
          yawMotorId)
{
}
TurretSubsystemMock::~TurretSubsystemMock() {}

XAxisSubsystemMock::XAxisSubsystemMock(
    aruwlib::Drivers *drivers,
    aruwlib::gpio::Digital::OutputPin pin)
    : engineer::XAxisSubsystem(drivers, pin)
{
}
XAxisSubsystemMock::~XAxisSubsystemMock() {}

XavierSerialMock::XavierSerialMock(aruwlib::Drivers *drivers) : serial::XavierSerial(drivers) {}
XavierSerialMock::~XavierSerialMock() {}
}  // namespace aruwsrc::mock
