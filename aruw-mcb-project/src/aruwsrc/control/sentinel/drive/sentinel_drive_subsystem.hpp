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

#include "aruwlib/communication/gpio/digital.hpp"
#include "aruwlib/control/chassis/i_chassis_subsystem.hpp"

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
#include "aruwlib/mock/dji_motor_mock.hpp"
#else
#include "aruwlib/motor/dji_motor.hpp"
#endif

#include "aruwlib/motor/m3508_constants.hpp"

#include "aruwsrc/control/chassis/power_limiter.hpp"
#include "modm/math/filter/pid.hpp"

#include "util_macros.hpp"

namespace aruwsrc::control::sentinel::drive
{
class SentinelDriveSubsystem : public aruwlib::control::chassis::iChassisSubsystem
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
        aruwlib::Drivers* drivers,
        aruwlib::gpio::Digital::InputPin leftLimitSwitch,
        aruwlib::gpio::Digital::InputPin rightLimitSwitch,
        aruwlib::gpio::Analog::Pin currentSensorPin,
        float pidP,
        float pidI,
        float pidD,
        float pidMaxErrorSum,
        float pidMaxOutput,
        float wheelRadius,
        float gearRatio,
        aruwlib::motor::MotorId leftMotorId,
        aruwlib::motor::MotorId rightMotorId,
        aruwlib::can::CanBus chassisCanBus);

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
    const aruwlib::gpio::Digital::InputPin LEFT_LIMIT_SWITCH;
    const aruwlib::gpio::Digital::InputPin RIGHT_LIMIT_SWITCH;
    const aruwlib::gpio::Analog::Pin CURRENT_SENSOR_PIN;

    // radius of the wheel in mm
    const float WHEEL_RADIUS;
    const float GEAR_RATIO;

    modm::Pid<float> velocityPidLeftWheel;

    modm::Pid<float> velocityPidRightWheel;

    float desiredRpm;
    float leftWheelZeroRailOffset = 0;
    float rightWheelZeroRailOffset = 0;

    void resetOffsetFromLimitSwitch();

    float distanceFromEncoder(aruwlib::motor::DjiMotor* motor);

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
public:
    aruwlib::mock::DjiMotorMock leftWheel;
    aruwlib::mock::DjiMotorMock rightWheel;

private:
#else
    aruwlib::motor::DjiMotor leftWheel;
    aruwlib::motor::DjiMotor rightWheel;
#endif

    aruwlib::motor::DjiMotor* chassisMotors[2];

    const aruwlib::motor::M3508Constants motorConstants;

    aruwsrc::chassis::PowerLimiter powerLimiter;
};

}  // namespace aruwsrc::control::sentinel::drive

#endif
