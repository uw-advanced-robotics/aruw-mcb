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

#ifndef SUBSYSTEM_SENTINEL_DRIVE_HPP_
#define SUBSYSTEM_SENTINEL_DRIVE_HPP_

#include "tap/communication/gpio/digital.hpp"
#include "tap/control/chassis/chassis_subsystem_interface.hpp"

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
#include "tap/mock/dji_motor_mock.hpp"
#else
#include "tap/motor/dji_motor.hpp"
#endif

#include "tap/control/chassis/power_limiter.hpp"
#include "tap/motor/m3508_constants.hpp"
#include "tap/util_macros.hpp"

#include "modm/math/filter/pid.hpp"

namespace aruwsrc::control::sentinel::drive
{
class SentinelDriveSubsystem : public tap::control::chassis::ChassisSubsystemInterface
{
public:
    /// @see power_limiter.hpp for what these mean
    static constexpr float MAX_ENERGY_BUFFER = 200.0f;
    static constexpr float ENERGY_BUFFER_LIMIT_THRESHOLD = 100.0f;
    static constexpr float ENERGY_BUFFER_CRIT_THRESHOLD = 0;
    static constexpr uint16_t POWER_CONSUMPTION_THRESHOLD = 5;
    static constexpr float CURRENT_ALLOCATED_FOR_ENERGY_BUFFER_LIMITING = 15000;

    // RMUL length of the rail, in mm
    static constexpr float RAIL_LENGTH = 2130;
    // Our length of the rail, in mm
    // static constexpr float RAIL_LENGTH = 1900;

    /**
     * Length of the sentinel, in mm
     */
    static constexpr float SENTINEL_LENGTH = 480;

    SentinelDriveSubsystem(
        tap::Drivers* drivers,
        tap::gpio::Digital::InputPin leftLimitSwitch,
        tap::gpio::Digital::InputPin rightLimitSwitch,
        tap::motor::MotorId leftMotorId = LEFT_MOTOR_ID,
        tap::motor::MotorId rightMotorId = RIGHT_MOTOR_ID,
        tap::gpio::Analog::Pin currentSensorPin = CURRENT_SENSOR_PIN);

    void initialize() override;

    /**
     * Returns absolute position of the sentinel, relative to the left end of the rail (when rail
     * is viewed from the front)
     */
    mockable float absolutePosition();

    mockable void setDesiredRpm(float desRpm);

    void refresh() override;

    void runHardwareTests() override;

    void onHardwareTestStart() override;

    void onHardwareTestComplete() override;

    const char* getName() override { return "Sentinel Drive"; }

    inline int getNumChassisMotors() const override { return 2; }

    inline int16_t getLeftFrontRpmActual() const override { return leftWheel.getShaftRPM(); }
    inline int16_t getLeftBackRpmActual() const override { return 0; }
    inline int16_t getRightFrontRpmActual() const override { return rightWheel.getShaftRPM(); }
    inline int16_t getRightBackRpmActual() const override { return 0; }

private:
    static constexpr tap::motor::MotorId LEFT_MOTOR_ID = tap::motor::MOTOR2;
    static constexpr tap::motor::MotorId RIGHT_MOTOR_ID = tap::motor::MOTOR1;
    static constexpr tap::can::CanBus CAN_BUS_MOTORS = tap::can::CanBus::CAN_BUS2;
    static constexpr tap::gpio::Analog::Pin CURRENT_SENSOR_PIN = tap::gpio::Analog::Pin::S;

    static constexpr float PID_P = 5.0f;
    static constexpr float PID_I = 0.0f;
    static constexpr float PID_D = 0.1f;
    static constexpr float PID_MAX_ERROR_SUM = 0.0f;
    static constexpr float PID_MAX_OUTPUT = 10000;

    // radius of the wheel in mm
    static constexpr float WHEEL_RADIUS = 35.0f;
    static constexpr float GEAR_RATIO = 19.0f;

    tap::gpio::Digital::InputPin leftLimitSwitch;
    tap::gpio::Digital::InputPin rightLimitSwitch;

    modm::Pid<float> velocityPidLeftWheel;

    modm::Pid<float> velocityPidRightWheel;

    float desiredRpm;
    float leftWheelZeroRailOffset = 0;
    float rightWheelZeroRailOffset = 0;

    void resetOffsetFromLimitSwitch();

    float distanceFromEncoder(tap::motor::DjiMotor* motor);

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
public:
    tap::mock::DjiMotorMock leftWheel;
    tap::mock::DjiMotorMock rightWheel;

private:
#else
    tap::motor::DjiMotor leftWheel;
    tap::motor::DjiMotor rightWheel;
#endif

    tap::motor::DjiMotor* chassisMotors[2];

    const tap::motor::M3508Constants motorConstants;

    tap::control::chassis::PowerLimiter powerLimiter;
};

}  // namespace aruwsrc::control::sentinel::drive

#endif
