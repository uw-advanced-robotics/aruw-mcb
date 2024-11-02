/*
 * Copyright (c) 2020-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "sentry_drive_subsystem.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/errors/create_errors.hpp"

#include "aruwsrc/communication/sensors/current/acs712_current_sensor_config.hpp"

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
#include "tap/mock/dji_motor_mock.hpp"
#else
#include "tap/motor/dji_motor.hpp"
#endif

#include "tap/drivers.hpp"

using namespace tap::gpio;

namespace aruwsrc::control::sentry::drive
{
SentryDriveSubsystem::SentryDriveSubsystem(
    tap::Drivers* drivers,
    tap::gpio::Digital::InputPin leftLimitSwitch,
    tap::gpio::Digital::InputPin rightLimitSwitch,
    tap::motor::MotorId leftMotorId,
    tap::gpio::Analog::Pin currentSensorPin)
    : tap::control::chassis::ChassisSubsystemInterface(drivers),
      leftLimitSwitch(leftLimitSwitch),
      rightLimitSwitch(rightLimitSwitch),
      velocityPidLeftWheel(PID_P, PID_I, PID_D, PID_MAX_ERROR_SUM, PID_MAX_OUTPUT),
      velocityPidRightWheel(PID_P, PID_I, PID_D, PID_MAX_ERROR_SUM, PID_MAX_OUTPUT),
      leftWheel(drivers, leftMotorId, CAN_BUS_MOTORS, false, "left sentry drive motor"),
      currentSensor(
          {&drivers->analog,
           currentSensorPin,
           aruwsrc::communication::sensors::current::ACS712_CURRENT_SENSOR_MV_PER_MA,
           aruwsrc::communication::sensors::current::ACS712_CURRENT_SENSOR_ZERO_MA,
           aruwsrc::communication::sensors::current::ACS712_CURRENT_SENSOR_LOW_PASS_ALPHA}),
      powerLimiter(
          drivers,
          &currentSensor,
          STARTING_ENERGY_BUFFER,
          ENERGY_BUFFER_LIMIT_THRESHOLD,
          ENERGY_BUFFER_CRIT_THRESHOLD)
{
    chassisMotors[0] = &leftWheel;
}

void SentryDriveSubsystem::initialize()
{
    if (leftLimitSwitch == rightLimitSwitch)
    {
        RAISE_ERROR(drivers, "identical left/right switch pins");
    }
    drivers->digital.configureInputPullMode(
        leftLimitSwitch,
        tap::gpio::Digital::InputPullMode::PullDown);
    drivers->digital.configureInputPullMode(
        rightLimitSwitch,
        tap::gpio::Digital::InputPullMode::PullDown);

    for (size_t i = 0; i < NUM_CHASSIS_MOTORS; i++)
    {
        chassisMotors[i]->initialize();
    }
}

void SentryDriveSubsystem::setDesiredRpm(float desRpm) { desiredRpm = desRpm; }

float SentryDriveSubsystem::getDesiredRpm() { return desiredRpm; }

modm::Matrix<float, 3, 1> SentryDriveSubsystem::getActualVelocityChassisRelative() const
{
    static constexpr float C = 2 * M_PI * WHEEL_RADIUS / 1000.0f;
    static constexpr float RPM_TO_MPS = C / (60.0f * GEAR_RATIO);
    float wheelVelRPM = leftWheel.getShaftRPM();

    modm::Matrix<float, 3, 1> wheelVelMat;
    wheelVelMat[0][0] = 0;
    wheelVelMat[0][1] = wheelVelRPM * RPM_TO_MPS;
    wheelVelMat[0][2] = 0;
    return wheelVelMat;
}

void SentryDriveSubsystem::refresh()
{
    // constantly poll the limit switches, resetting offset if needed
    this->resetOffsetFromLimitSwitch();

    float speedReductionScalar =
        computeEndOfRailSpeedReductionScalar(this->desiredRpm, this->getAbsolutePosition());
    float scaledDesiredRpm = speedReductionScalar * this->desiredRpm;

    this->velocityPidLeftWheel.update(scaledDesiredRpm - this->leftWheel.getShaftRPM());
    this->leftWheel.setDesiredOutput(this->velocityPidLeftWheel.getValue());
    currentSensor.update();
    float powerLimitFrac = powerLimiter.getPowerLimitRatio();

    if (tap::algorithms::compareFloatClose(1.0f, powerLimitFrac, 1E-5))
    {
        return;
    }

    for (size_t i = 0; i < MODM_ARRAY_SIZE(chassisMotors); i++)
    {
        chassisMotors[i]->setDesiredOutput(chassisMotors[i]->getOutputDesired() * powerLimitFrac);
    }
}

float SentryDriveSubsystem::getAbsolutePosition() const
{
    if (!leftWheel.isMotorOnline())
    {
        RAISE_ERROR(drivers, "left sentry drive motor offline");
        return 0;
    }
    return distanceFromEncoder(&leftWheel) - leftWheelZeroRailOffset;
}

// Resets the encoder offset used to determine position of the sentry on the rail depending on
// which limit switch is hit. If neither limit switch is hit, no-op. Left limit switch indicates
// being at the start of the rail, right limit switch indicates end of rail.
void SentryDriveSubsystem::resetOffsetFromLimitSwitch()
{
    // DigitalPin where limit switch is placed

    if (drivers->digital.read(leftLimitSwitch))
    {
        leftWheelZeroRailOffset = distanceFromEncoder(&leftWheel) - RAIL_LENGTH + SENTRY_LENGTH;
    }
    else if (drivers->digital.read(rightLimitSwitch))
    {
        leftWheelZeroRailOffset = distanceFromEncoder(&leftWheel);
    }
}

// Returns the distance covered by the sentry wheel on the rail
// with respect to the encoders
// Equation used: Arc Length = Angle * radius
// Here we get the shaft angle from the getEncoderUnwrapped function
float SentryDriveSubsystem::distanceFromEncoder(const tap::motor::DjiMotor* motor) const
{
    float unwrappedAngle = motor->getEncoderUnwrapped();
    float numberOfRotations = unwrappedAngle / (tap::motor::DjiMotor::ENC_RESOLUTION);
    return numberOfRotations * 2.0f * M_PI * WHEEL_RADIUS / GEAR_RATIO;
}
}  // namespace aruwsrc::control::sentry::drive
