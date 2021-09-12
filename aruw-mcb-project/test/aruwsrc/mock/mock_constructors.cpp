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
#include "imu_rx_listener_mock.hpp"
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
    tap::Drivers *drivers,
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
    tap::Drivers *drivers,
    chassis::ChassisSubsystem *chassis,
    tap::control::turret::TurretSubsystemInterface *turret)
    : BeybladeCommand(drivers, chassis, turret)
{
}
BeybladeCommandMock::~BeybladeCommandMock() {}

ChassisDriveCommandMock::ChassisDriveCommandMock(tap::Drivers *d, chassis::ChassisSubsystem *cs)
    : chassis::ChassisDriveCommand(d, cs)
{
}
ChassisDriveCommandMock::~ChassisDriveCommandMock() {}

ChassisSubsystemMock::ChassisSubsystemMock(tap::Drivers *drivers) : ChassisSubsystem(drivers) {}
ChassisSubsystemMock::~ChassisSubsystemMock() {}

FrictionWheelSubsystemMock::FrictionWheelSubsystemMock(tap::Drivers *drivers)
    : FrictionWheelSubsystem(drivers)
{
}
FrictionWheelSubsystemMock::~FrictionWheelSubsystemMock() {}

GrabberSubsystemMock::GrabberSubsystemMock(tap::Drivers *drivers, tap::gpio::Digital::OutputPin pin)
    : engineer::GrabberSubsystem(drivers, pin)
{
}
GrabberSubsystemMock::~GrabberSubsystemMock() {}

OledDisplayMock::OledDisplayMock(tap::Drivers *drivers) : display::OledDisplay(drivers) {}
OledDisplayMock::~OledDisplayMock() {}

ImuRxListenerMock::ImuRxListenerMock(tap::Drivers *drivers) : can::ImuRxListener(drivers) {}
ImuRxListenerMock::~ImuRxListenerMock() {}

HopperSubsystemMock::HopperSubsystemMock(
    tap::Drivers *drivers,
    tap::gpio::Pwm::Pin pwmPin,
    float open,
    float close,
    float pwmRampSpeed)
    : control::HopperSubsystem(drivers, pwmPin, open, close, pwmRampSpeed)
{
}
HopperSubsystemMock::~HopperSubsystemMock() {}

SentinelDriveSubsystemMock::SentinelDriveSubsystemMock(
    tap::Drivers *drivers,
    tap::gpio::Digital::InputPin leftLimitSwitch,
    tap::gpio::Digital::InputPin rightLimitSwitch)
    : control::sentinel::drive::SentinelDriveSubsystem(drivers, leftLimitSwitch, rightLimitSwitch)
{
}
SentinelDriveSubsystemMock::~SentinelDriveSubsystemMock() {}

SentinelSwitcherSubsystemMock::SentinelSwitcherSubsystemMock(
    tap::Drivers *drivers,
    tap::gpio::Pwm::Pin switcherServoPin)
    : control::sentinel::firing::SentinelSwitcherSubsystem(drivers, switcherServoPin)
{
}
SentinelSwitcherSubsystemMock::~SentinelSwitcherSubsystemMock() {}

TowSubsystemMock::TowSubsystemMock(
    tap::Drivers *drivers,
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

TurretSubsystemMock::TurretSubsystemMock(tap::Drivers *drivers) : TurretSubsystem(drivers) {}
TurretSubsystemMock::~TurretSubsystemMock() {}

XAxisSubsystemMock::XAxisSubsystemMock(tap::Drivers *drivers, tap::gpio::Digital::OutputPin pin)
    : engineer::XAxisSubsystem(drivers, pin)
{
}
XAxisSubsystemMock::~XAxisSubsystemMock() {}

XavierSerialMock::XavierSerialMock(tap::Drivers *drivers) : serial::XavierSerial(drivers) {}
XavierSerialMock::~XavierSerialMock() {}
}  // namespace aruwsrc::mock
