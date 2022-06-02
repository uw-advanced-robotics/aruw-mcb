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

#include "sentinel_drive_subsystem.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/errors/create_errors.hpp"

#include "aruwsrc/communication/sensors/current/acs712_current_sensor_config.hpp"
#include "aruwsrc/drivers.hpp"
#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
#include "tap/mock/dji_motor_mock.hpp"
#else
#include "tap/motor/dji_motor.hpp"
#endif

using namespace tap::gpio;

namespace aruwsrc::control::sentinel::drive
{
SentinelDriveSubsystem::SentinelDriveSubsystem(
    aruwsrc::Drivers* drivers,
    tap::gpio::Digital::InputPin leftLimitSwitch,
    tap::gpio::Digital::InputPin rightLimitSwitch,
    tap::motor::MotorId leftMotorId,
#if defined(TARGET_SENTINEL_2021)
    tap::motor::MotorId rightMotorId,
#endif
    tap::gpio::Analog::Pin currentSensorPin)
    : tap::control::chassis::ChassisSubsystemInterface(drivers),
      leftLimitSwitch(leftLimitSwitch),
      rightLimitSwitch(rightLimitSwitch),
      velocityPidLeftWheel(PID_P, PID_I, PID_D, PID_MAX_ERROR_SUM, PID_MAX_OUTPUT),
      velocityPidRightWheel(PID_P, PID_I, PID_D, PID_MAX_ERROR_SUM, PID_MAX_OUTPUT),
      leftWheel(drivers, leftMotorId, CAN_BUS_MOTORS, false, "left sentinel drive motor"),
#if defined(TARGET_SENTINEL_2021)
      rightWheel(drivers, rightMotorId, CAN_BUS_MOTORS, false, "right sentinel drive motor"),
#endif
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
#if defined(TARGET_SENTINEL_2021)
    chassisMotors[1] = &rightWheel;
#endif
}

void SentinelDriveSubsystem::initialize()
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

void SentinelDriveSubsystem::setDesiredRpm(float desRpm) { desiredRpm = desRpm; }

float SentinelDriveSubsystem::getDesiredRpm() { return desiredRpm; }

modm::Matrix<float, 3, 1> SentinelDriveSubsystem::getActualVelocityChassisRelative() const
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

void SentinelDriveSubsystem::refresh()
{
    // constantly poll the limit switches, resetting offset if needed
    resetOffsetFromLimitSwitch();

    velocityPidLeftWheel.update(desiredRpm - leftWheel.getShaftRPM());
    leftWheel.setDesiredOutput(velocityPidLeftWheel.getValue());
#if defined(TARGET_SENTINEL_2021)
    velocityPidRightWheel.update(desiredRpm - rightWheel.getShaftRPM());
    rightWheel.setDesiredOutput(velocityPidRightWheel.getValue());
#endif
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

float SentinelDriveSubsystem::absolutePosition()
{
#if defined(TARGET_SENTINEL_2021)
    float leftPosition = distanceFromEncoder(&leftWheel) - leftWheelZeroRailOffset;
    float rightPosition = distanceFromEncoder(&rightWheel) - rightWheelZeroRailOffset;
    float average = 0.0f;
    if (leftWheel.isMotorOnline() && rightWheel.isMotorOnline())
    {
        average = (leftPosition + rightPosition) / 2.0f;
    }
    else if (leftWheel.isMotorOnline())
    {
        RAISE_ERROR(drivers, "right sentinel drive motor offline");
        average = leftPosition;
    }
    else if (rightWheel.isMotorOnline())
    {
        RAISE_ERROR(drivers, "left sentinel drive motor offline");
        average = rightPosition;
    }
    else
    {
        RAISE_ERROR(drivers, "both sentinel drive motors offline");
        average = 0.0f;
    }
    return average;
#else
    if (!leftWheel.isMotorOnline())
    {
        RAISE_ERROR(drivers, "left sentinel drive motor offline");
        return 0;
    }
    return distanceFromEncoder(&leftWheel) - leftWheelZeroRailOffset;
#endif
}

// Resets the encoder offset used to determine position of the sentinel on the rail depending on
// which limit switch is hit. If neither limit switch is hit, no-op. Left limit switch indicates
// being at the start of the rail, right limit switch indicates end of rail.
void SentinelDriveSubsystem::resetOffsetFromLimitSwitch()
{
    // DigitalPin where limit switch is placed

    if (drivers->digital.read(leftLimitSwitch))
    {
        leftWheelZeroRailOffset = distanceFromEncoder(&leftWheel) - RAIL_LENGTH + SENTINEL_LENGTH;
#if defined(TARGET_SENTINEL_2021)
        rightWheelZeroRailOffset = distanceFromEncoder(&rightWheel) - RAIL_LENGTH + SENTINEL_LENGTH;
#endif
    }
    else if (drivers->digital.read(rightLimitSwitch))
    {
        leftWheelZeroRailOffset = distanceFromEncoder(&leftWheel);
#if defined(TARGET_SENTINEL_2021)
        rightWheelZeroRailOffset = distanceFromEncoder(&rightWheel);
#endif
    }
}

// Returns the distance covered by the sentinel wheel on the rail
// with respect to the encoders
// Equation used: Arc Length = Angle * radius
// Here we get the shaft angle from the getEncoderUnwrapped function
float SentinelDriveSubsystem::distanceFromEncoder(tap::motor::DjiMotor* motor)
{
    float unwrappedAngle = motor->getEncoderUnwrapped();
    float numberOfRotations = unwrappedAngle / (tap::motor::DjiMotor::ENC_RESOLUTION);
    return numberOfRotations * 2.0f * M_PI * WHEEL_RADIUS / GEAR_RATIO;
}

void SentinelDriveSubsystem::runHardwareTests() {}

void SentinelDriveSubsystem::onHardwareTestStart() {}

void SentinelDriveSubsystem::onHardwareTestComplete() {}

}  // namespace aruwsrc::control::sentinel::drive
