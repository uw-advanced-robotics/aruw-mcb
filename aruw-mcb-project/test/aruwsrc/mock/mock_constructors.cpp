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
#include "turret_mcb_can_comm_mock.hpp"
#include "turret_subsystem_mock.hpp"
#include "x_axis_subsystem_mock.hpp"
#include "legacy_vision_coprocessor_mock.hpp"

// A file for listing all mock constructors and destructors since doing
// so in a source file allows for faster compilation than defining constructors
// in the headers
namespace aruwsrc::mock
{
AgitatorSubsystemMock::AgitatorSubsystemMock(
    aruwsrc::Drivers *drivers,
    float kp,
    float ki,
    float kd,
    float maxIAccum,
    float maxOutput,
    float agitatorGearRatio,
    tap::motor::MotorId agitatorMotorId,
    tap::can::CanBus agitatorCanBusId,
    bool isAgitatorInverted)
    : AgitatorSubsystem(
          drivers,
          kp,
          ki,
          kd,
          maxIAccum,
          maxOutput,
          agitatorGearRatio,
          agitatorMotorId,
          agitatorCanBusId,
          isAgitatorInverted)
{
    ON_CALL(*this, isOnline).WillByDefault(testing::Return(true));
}
AgitatorSubsystemMock::~AgitatorSubsystemMock() {}

BeybladeCommandMock::BeybladeCommandMock(
    aruwsrc::Drivers *drivers,
    chassis::ChassisSubsystem *chassis,
    tap::control::turret::TurretSubsystemInterface *turret)
    : BeybladeCommand(drivers, chassis, turret)
{
}
BeybladeCommandMock::~BeybladeCommandMock() {}

ChassisDriveCommandMock::ChassisDriveCommandMock(aruwsrc::Drivers *d, chassis::ChassisSubsystem *cs)
    : chassis::ChassisDriveCommand(d, cs)
{
}
ChassisDriveCommandMock::~ChassisDriveCommandMock() {}

ChassisSubsystemMock::ChassisSubsystemMock(aruwsrc::Drivers *drivers) : ChassisSubsystem(drivers) {}
ChassisSubsystemMock::~ChassisSubsystemMock() {}

FrictionWheelSubsystemMock::FrictionWheelSubsystemMock(aruwsrc::Drivers *drivers)
    : FrictionWheelSubsystem(drivers)
{
}
FrictionWheelSubsystemMock::~FrictionWheelSubsystemMock() {}

GrabberSubsystemMock::GrabberSubsystemMock(
    aruwsrc::Drivers *drivers,
    tap::gpio::Digital::OutputPin pin)
    : engineer::GrabberSubsystem(drivers, pin)
{
}
GrabberSubsystemMock::~GrabberSubsystemMock() {}

OledDisplayMock::OledDisplayMock(aruwsrc::Drivers *drivers) : display::OledDisplay(drivers) {}
OledDisplayMock::~OledDisplayMock() {}

TurretMCBCanCommMock::TurretMCBCanCommMock(aruwsrc::Drivers *drivers)
    : can::TurretMCBCanComm(drivers)
{
}
TurretMCBCanCommMock::~TurretMCBCanCommMock() {}

HopperSubsystemMock::HopperSubsystemMock(
    aruwsrc::Drivers *drivers,
    tap::gpio::Pwm::Pin pwmPin,
    float open,
    float close,
    float pwmRampSpeed)
    : control::HopperSubsystem(drivers, pwmPin, open, close, pwmRampSpeed)
{
}
HopperSubsystemMock::~HopperSubsystemMock() {}

SentinelDriveSubsystemMock::SentinelDriveSubsystemMock(
    aruwsrc::Drivers *drivers,
    tap::gpio::Digital::InputPin leftLimitSwitch,
    tap::gpio::Digital::InputPin rightLimitSwitch)
    : control::sentinel::drive::SentinelDriveSubsystem(drivers, leftLimitSwitch, rightLimitSwitch)
{
}
SentinelDriveSubsystemMock::~SentinelDriveSubsystemMock() {}

SentinelSwitcherSubsystemMock::SentinelSwitcherSubsystemMock(
    aruwsrc::Drivers *drivers,
    tap::gpio::Pwm::Pin switcherServoPin)
    : control::sentinel::firing::SentinelSwitcherSubsystem(drivers, switcherServoPin)
{
}
SentinelSwitcherSubsystemMock::~SentinelSwitcherSubsystemMock() {}

TowSubsystemMock::TowSubsystemMock(
    aruwsrc::Drivers *drivers,
    tap::gpio::Digital::OutputPin leftTowPin,
    tap::gpio::Digital::OutputPin rightTowPin,
    tap::gpio::Digital::InputPin leftTowLimitSwitchPin,
    tap::gpio::Digital::InputPin rightTowLimitSwitchPin)
    : aruwsrc::engineer::TowSubsystem(
          drivers,
          leftTowPin,
          rightTowPin,
          leftTowLimitSwitchPin,
          rightTowLimitSwitchPin)
{
}
TowSubsystemMock::~TowSubsystemMock() {}

TurretSubsystemMock::TurretSubsystemMock(aruwsrc::Drivers *drivers)
    : TurretSubsystem(drivers, nullptr, nullptr)
{
}
TurretSubsystemMock::~TurretSubsystemMock() {}

XAxisSubsystemMock::XAxisSubsystemMock(aruwsrc::Drivers *drivers, tap::gpio::Digital::OutputPin pin)
    : engineer::XAxisSubsystem(drivers, pin)
{
}
XAxisSubsystemMock::~XAxisSubsystemMock() {}

LegacyVisionCoprocessorMock::LegacyVisionCoprocessorMock(aruwsrc::Drivers *drivers) : serial::LegacyVisionCoprocessor(drivers) {}
LegacyVisionCoprocessorMock::~LegacyVisionCoprocessorMock() {}
}  // namespace aruwsrc::mock
