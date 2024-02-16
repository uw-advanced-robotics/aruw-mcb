/*
 * Copyright (c) 2020-2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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
#include "cv_on_target_governor_mock.hpp"
#include "friction_wheel_subsystem_mock.hpp"
#include "grabber_subsystem_mock.hpp"
#include "hopper_subsystem_mock.hpp"
#include "mecanum_chassis_subsystem_mock.hpp"
#include "oled_display_mock.hpp"
#include "otto_ballistics_solver_mock.hpp"
#include "referee_feedback_friction_wheel_subsystem_mock.hpp"
#include "robot_turret_subsystem_mock.hpp"
#include "sentry_drive_subsystem_mock.hpp"
#include "sentry_request_subsystem_mock.hpp"
#include "swerve_chassis_subsystem_mock.hpp"
#include "swerve_module_mock.hpp"
#include "tow_subsystem_mock.hpp"
#include "turret_controller_interface_mock.hpp"
#include "turret_cv_command_mock.hpp"
#include "turret_mcb_can_comm_mock.hpp"
#include "turret_motor_mock.hpp"
#include "turret_subsystem_mock.hpp"
#include "vision_coprocessor_mock.hpp"
#include "x_axis_subsystem_mock.hpp"
#include "x_drive_chassis_subsystem_mock.hpp"

// A file for listing all mock constructors and destructors since doing
// so in a source file allows for faster compilation than defining constructors
// in the headers
namespace aruwsrc::mock
{
AgitatorSubsystemMock::AgitatorSubsystemMock(
    tap::Drivers *drivers,
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

// BeybladeCommandMock::BeybladeCommandMock(
//     tap::Drivers *drivers,
//     chassis::MecanumChassisSubsystem *chassis,
//     aruwsrc::control::turret::TurretMotor *yawMotor,
//     aruwsrc::control::ControlOperatorInterface &operatorInterface)
//     : BeybladeCommand(drivers, chassis, yawMotor, operatorInterface)
// {
// }
// BeybladeCommandMock::~BeybladeCommandMock() {}

// ChassisDriveCommandMock::ChassisDriveCommandMock(
//     tap::Drivers *d,
//     aruwsrc::control::ControlOperatorInterface *operatorInterface,
//     chassis::MecanumChassisSubsystem *cs)
//     : chassis::ChassisDriveCommand(d, operatorInterface, cs)
// {
// }
// ChassisDriveCommandMock::~ChassisDriveCommandMock() {}

// MecanumChassisSubsystemMock::MecanumChassisSubsystemMock(
//     tap::Drivers *drivers,
//     tap::communication::sensors::current::CurrentSensorInterface *currentSensor)
//     : MecanumChassisSubsystem(drivers, currentSensor)
// {
// }
// MecanumChassisSubsystemMock::~MecanumChassisSubsystemMock() {}

// XDriveChassisSubsystemMock::XDriveChassisSubsystemMock(
//     tap::Drivers *drivers,
//     tap::communication::sensors::current::CurrentSensorInterface *currentSensor)
//     : XDriveChassisSubsystem(drivers, currentSensor)
// {
// }
// XDriveChassisSubsystemMock::~XDriveChassisSubsystemMock() {}

// SwerveChassisSubsystemMock::SwerveChassisSubsystemMock(
//     tap::Drivers *drivers,
//     tap::communication::sensors::current::CurrentSensorInterface *currentSensor,
//     testing::NiceMock<aruwsrc::mock::SwerveModuleMock> *lf,
//     testing::NiceMock<aruwsrc::mock::SwerveModuleMock> *rf,
//     testing::NiceMock<aruwsrc::mock::SwerveModuleMock> *lb,
//     testing::NiceMock<aruwsrc::mock::SwerveModuleMock> *rb)
//     : SwerveChassisSubsystem(drivers, currentSensor, lf, rf, lb, rb, SWERVE_FORWARD_MATRIX)
// {
// }
// SwerveChassisSubsystemMock::~SwerveChassisSubsystemMock() {}

// SwerveModuleMock::SwerveModuleMock(
//     testing::NiceMock<tap::mock::DjiMotorMock> &driMotor,
//     testing::NiceMock<tap::mock::DjiMotorMock> &aziMotor,
//     aruwsrc::chassis::SwerveModuleConfig &config)
//     : SwerveModule(aziMotor, driMotor, config)
// {
// }
// SwerveModuleMock::~SwerveModuleMock() {}

FrictionWheelSubsystemMock::FrictionWheelSubsystemMock(tap::Drivers *drivers)
    : FrictionWheelSubsystem(
          drivers,
          tap::motor::MOTOR1,
          tap::motor::MOTOR2,
          tap::can::CanBus::CAN_BUS1,
          nullptr)
{
}
FrictionWheelSubsystemMock::~FrictionWheelSubsystemMock() {}

RefereeFeedbackFrictionWheelSubsystemMock::RefereeFeedbackFrictionWheelSubsystemMock(
    tap::Drivers *drivers)
    : RefereeFeedbackFrictionWheelSubsystem<10>(
          drivers,
          tap::motor::MOTOR1,
          tap::motor::MOTOR2,
          tap::can::CanBus::CAN_BUS1,
          nullptr,
          tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_17MM_1)
{
}
RefereeFeedbackFrictionWheelSubsystemMock::~RefereeFeedbackFrictionWheelSubsystemMock() {}

GrabberSubsystemMock::GrabberSubsystemMock(tap::Drivers *drivers, tap::gpio::Digital::OutputPin pin)
    : engineer::GrabberSubsystem(drivers, pin)
{
}
GrabberSubsystemMock::~GrabberSubsystemMock() {}

OledDisplayMock::OledDisplayMock(
    tap::Drivers *drivers,
    aruwsrc::serial::VisionCoprocessor *vc,
    can::TurretMCBCanComm *turretMCBCanCommBus1,
    can::TurretMCBCanComm *turretMCBCanCommBus2)
    : display::OledDisplay(drivers, vc, turretMCBCanCommBus1, turretMCBCanCommBus2)
{
}
OledDisplayMock::~OledDisplayMock() {}

TurretMCBCanCommMock::TurretMCBCanCommMock(tap::Drivers *drivers, tap::can::CanBus canBus)
    : can::TurretMCBCanComm(drivers, canBus)
{
}
TurretMCBCanCommMock::~TurretMCBCanCommMock() {}

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

SentryDriveSubsystemMock::SentryDriveSubsystemMock(
    tap::Drivers *drivers,
    tap::gpio::Digital::InputPin leftLimitSwitch,
    tap::gpio::Digital::InputPin rightLimitSwitch)
    : control::sentry::drive::SentryDriveSubsystem(drivers, leftLimitSwitch, rightLimitSwitch)
{
}
SentryDriveSubsystemMock::~SentryDriveSubsystemMock() {}

SentryRequestSubsystemMock::SentryRequestSubsystemMock(tap::Drivers *drivers)
    : SentryRequestSubsystem(drivers)
{
}
SentryRequestSubsystemMock::~SentryRequestSubsystemMock() {}

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

TurretSubsystemMock::TurretSubsystemMock(tap::Drivers *drivers)
    : TurretSubsystem(drivers, &m, &m, MOTOR_CONFIG, MOTOR_CONFIG, nullptr)
{
}
TurretSubsystemMock::~TurretSubsystemMock() {}

RobotTurretSubsystemMock::RobotTurretSubsystemMock(tap::Drivers *drivers)
    : RobotTurretSubsystem(drivers, &m, &m, MOTOR_CONFIG, MOTOR_CONFIG, nullptr)
{
}
RobotTurretSubsystemMock::~RobotTurretSubsystemMock() {}

XAxisSubsystemMock::XAxisSubsystemMock(tap::Drivers *drivers, tap::gpio::Digital::OutputPin pin)
    : engineer::XAxisSubsystem(drivers, pin)
{
}
XAxisSubsystemMock::~XAxisSubsystemMock() {}

VisionCoprocessorMock::VisionCoprocessorMock(tap::Drivers *drivers)
    : serial::VisionCoprocessor(drivers)
{
}
VisionCoprocessorMock::~VisionCoprocessorMock() {}

TurretMotorMock::TurretMotorMock(
    tap::motor::MotorInterface *motor,
    const control::turret::TurretMotorConfig &motorConfig)
    : aruwsrc::control::turret::TurretMotor(motor, motorConfig)
{
    ON_CALL(*this, getValidMinError)
        .WillByDefault([&](const float setpoint, const float measurement) {
            return tap::algorithms::ContiguousFloat(measurement, 0, M_TWOPI).difference(setpoint);
        });
    ON_CALL(*this, getValidChassisMeasurementError).WillByDefault([&]() {
        return getValidMinError(
            getChassisFrameSetpoint(),
            getChassisFrameMeasuredAngle().getValue());
    });
    ON_CALL(*this, getConfig).WillByDefault(testing::ReturnRef(defaultConfig));
}
TurretMotorMock::~TurretMotorMock() {}

TurretCVCommandMock::TurretCVCommandMock(
    serial::VisionCoprocessor *visionCoprocessor,
    control::ControlOperatorInterface *controlOperatorInterface,
    aruwsrc::control::turret::RobotTurretSubsystem *turretSubsystem,
    aruwsrc::control::turret::algorithms::TurretYawControllerInterface *yawController,
    aruwsrc::control::turret::algorithms::TurretPitchControllerInterface *pitchController,
    aruwsrc::algorithms::OttoBallisticsSolver *ballisticsSolver,
    const float userPitchInputScalar,
    const float userYawInputScalar,
    uint8_t turretID)
    : aruwsrc::control::turret::cv::TurretCVCommand(
          visionCoprocessor,
          controlOperatorInterface,
          turretSubsystem,
          yawController,
          pitchController,
          ballisticsSolver,
          userPitchInputScalar,
          userYawInputScalar,
          turretID)
{
}
TurretCVCommandMock::~TurretCVCommandMock() {}

OttoBallisticsSolverMock::OttoBallisticsSolverMock(
    const aruwsrc::serial::VisionCoprocessor &visionCoprocessor,
    const tap::algorithms::odometry::Odometry2DInterface &odometryInterface,
    const control::turret::RobotTurretSubsystem &turretSubsystem,
    const control::launcher::LaunchSpeedPredictorInterface &frictionWheels,
    const float defaultLaunchSpeed,
    const uint8_t turretID)
    : aruwsrc::algorithms::OttoBallisticsSolver(
          visionCoprocessor,
          odometryInterface,
          turretSubsystem,
          frictionWheels,
          defaultLaunchSpeed,
          turretID){};

OttoBallisticsSolverMock::~OttoBallisticsSolverMock(){};

TurretControllerInterfaceMock::TurretControllerInterfaceMock(
    aruwsrc::control::turret::TurretMotor &turretMotor)
    : aruwsrc::control::turret::algorithms::TurretControllerInterface(turretMotor)
{
}
TurretControllerInterfaceMock::~TurretControllerInterfaceMock() {}

CvOnTargetGovernorMock::CvOnTargetGovernorMock(
    tap::Drivers *drivers,
    aruwsrc::serial::VisionCoprocessor &visionCoprocessor,
    aruwsrc::control::turret::cv::TurretCVCommandInterface &turretCVCommand,
    aruwsrc::control::governor::AutoAimLaunchTimer &launchTimer,
    aruwsrc::control::governor::CvOnTargetGovernorMode mode)
    : aruwsrc::control::governor::CvOnTargetGovernor(
          drivers,
          visionCoprocessor,
          turretCVCommand,
          launchTimer,
          mode)
{
}

}  // namespace aruwsrc::mock
