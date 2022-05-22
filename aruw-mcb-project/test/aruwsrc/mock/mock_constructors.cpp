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
#include "referee_feedback_friction_wheel_subsystem_mock.hpp"
#include "sentinel_drive_subsystem_mock.hpp"
#include "sentinel_request_subsystem_mock.hpp"
#include "tow_subsystem_mock.hpp"
#include "turret_cv_command_mock.hpp"
#include "turret_mcb_can_comm_mock.hpp"
#include "turret_subsystem_mock.hpp"
#include "x_axis_subsystem_mock.hpp"

// A file for listing all mock constructors and destructors since doing
// so in a source file allows for faster compilation than defining constructors
// in the headers
namespace aruwsrc::mock
{
AgitatorSubsystemMock::AgitatorSubsystemMock(
    aruwsrc::Drivers *drivers,
    const tap::algorithms::SmoothPidConfig &pidConfig,
    float agitatorGearRatio,
    tap::motor::MotorId agitatorMotorId,
    tap::can::CanBus agitatorCanBusId,
    bool isAgitatorInverted,
    float jammingDistance,
    uint32_t jammingTime,
    bool jamLogicEnabled)
    : AgitatorSubsystem(
          drivers,
          pidConfig,
          agitatorGearRatio,
          agitatorMotorId,
          agitatorCanBusId,
          isAgitatorInverted,
          jammingDistance,
          jammingTime,
          jamLogicEnabled)
{
    ON_CALL(*this, isOnline).WillByDefault(testing::Return(true));
}
AgitatorSubsystemMock::~AgitatorSubsystemMock() {}

BeybladeCommandMock::BeybladeCommandMock(
    aruwsrc::Drivers *drivers,
    chassis::ChassisSubsystem *chassis,
    aruwsrc::control::turret::TurretMotor *yawMotor)
    : BeybladeCommand(drivers, chassis, yawMotor)
{
}
BeybladeCommandMock::~BeybladeCommandMock() {}

ChassisDriveCommandMock::ChassisDriveCommandMock(aruwsrc::Drivers *d, chassis::ChassisSubsystem *cs)
    : chassis::ChassisDriveCommand(d, cs)
{
}
ChassisDriveCommandMock::~ChassisDriveCommandMock() {}

ChassisSubsystemMock::ChassisSubsystemMock(aruwsrc::Drivers *drivers)
    : ChassisSubsystem(drivers, chassis::ChassisSubsystem::ChassisType::MECANUM)
{
}
ChassisSubsystemMock::~ChassisSubsystemMock() {}

FrictionWheelSubsystemMock::FrictionWheelSubsystemMock(aruwsrc::Drivers *drivers)
    : FrictionWheelSubsystem(
          drivers,
          tap::motor::MOTOR1,
          tap::motor::MOTOR2,
          tap::can::CanBus::CAN_BUS1)
{
}
FrictionWheelSubsystemMock::~FrictionWheelSubsystemMock() {}

RefereeFeedbackFrictionWheelSubsystemMock::RefereeFeedbackFrictionWheelSubsystemMock(
    aruwsrc::Drivers *drivers)
    : RefereeFeedbackFrictionWheelSubsystem<10>(
          drivers,
          tap::motor::MOTOR1,
          tap::motor::MOTOR2,
          tap::can::CanBus::CAN_BUS1,
          tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_17MM_1)
{
}
RefereeFeedbackFrictionWheelSubsystemMock::~RefereeFeedbackFrictionWheelSubsystemMock() {}

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

SentinelRequestSubsystemMock::SentinelRequestSubsystemMock(aruwsrc::Drivers *drivers)
    : SentinelRequestSubsystem(drivers)
{
}
SentinelRequestSubsystemMock::~SentinelRequestSubsystemMock() {}

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
    : TurretSubsystem(drivers, &m, &m, MOTOR_CONFIG, MOTOR_CONFIG)
{
}
TurretSubsystemMock::~TurretSubsystemMock() {}

XAxisSubsystemMock::XAxisSubsystemMock(aruwsrc::Drivers *drivers, tap::gpio::Digital::OutputPin pin)
    : engineer::XAxisSubsystem(drivers, pin)
{
}
XAxisSubsystemMock::~XAxisSubsystemMock() {}

VisionCoprocessorMock::VisionCoprocessorMock(aruwsrc::Drivers *drivers)
    : serial::VisionCoprocessor(drivers)
{
}
VisionCoprocessorMock::~VisionCoprocessorMock() {}

TurretMotorMock::TurretMotorMock(
    tap::motor::MotorInterface *motor,
    const control::turret::TurretMotorConfig &motorConfig)
    : aruwsrc::control::turret::TurretMotor(motor, motorConfig)
{
    ON_CALL(*this, getValidMinError).WillByDefault([&](const float measurement) {
        return tap::algorithms::ContiguousFloat(measurement, 0, M_TWOPI)
            .difference(getChassisFrameSetpoint());
    });
    ON_CALL(*this, getValidChassisMeasurementError).WillByDefault([&]() {
        return getValidMinError(getChassisFrameMeasuredAngle().getValue());
    });
    ON_CALL(*this, getConfig).WillByDefault(testing::ReturnRef(defaultConfig));
}
TurretMotorMock::~TurretMotorMock() {}

TurretCVCommandMock::TurretCVCommandMock(
    aruwsrc::Drivers *drivers,
    aruwsrc::control::turret::TurretSubsystem *turretSubsystem,
    aruwsrc::control::turret::algorithms::TurretYawControllerInterface *yawController,
    aruwsrc::control::turret::algorithms::TurretPitchControllerInterface *pitchController,
    const float userPitchInputScalar,
    const float userYawInputScalar,
    uint8_t turretID)
    : aruwsrc::control::turret::cv::TurretCVCommand(
          drivers,
          turretSubsystem,
          yawController,
          pitchController,
          userPitchInputScalar,
          userYawInputScalar,
          turretID)
{
}
TurretCVCommandMock::~TurretCVCommandMock() {}
}  // namespace aruwsrc::mock
