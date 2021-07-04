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
#include "aruwlib/control/chassis/chassis_subsystem_interface.hpp"

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
#include "aruwlib/mock/dji_motor_mock.hpp"
#else
#include "aruwlib/motor/dji_motor.hpp"
#endif

#include "aruwlib/control/chassis/power_limiter.hpp"
#include "aruwlib/motor/m3508_constants.hpp"
#include "aruwlib/util_macros.hpp"

#include "modm/math/filter/pid.hpp"

namespace aruwsrc::control::sentinel::drive
{
class SentinelDriveSubsystem : public aruwlib::control::chassis::ChassisSubsystemInterface
{
public:
    /**
     * @brief Constructs a SentinelDriveSubsystem with the specified parameters.
     *
     * @param[in] drivers Pointer to a drivers singleton object.
     * @param[in] leftLimitSwitch Left limit switch connected to the side of the chassis
     *      and is triggered when the sentinel reaches the end of the rail.
     * @param[in] rightLimitSwitch Right limit switch connected to the side of the chassis
     *      and is triggered when the sentinel reaches the end of the rail.
     * @param[in] currentSensorPin The analog input pin that the chassis current sensor is connected
     *      to.
     * @param[in] pidP Proportional term for chassis wheel velocity PID controller.
     * @param[in] pidI Integral term for chassis wheel velocity PID controller
     * @param[in] pidD Derivative term for chassis wheel velocity PID controller.
     * @param[in] pidMaxErrorSum Max integral sum for chassis wheel velocity PID controller.
     * @param[in] pidMaxOutput This max output is measured in the c620 robomaster translated
     *      current. Per the datasheet, the controllable current range is -16384 ~ 0 ~ 16384. The
     *      corresponding speed controller output torque current range is -20 ~ 0 ~ 20 A.
     * @param[in] wheelRadius Wheel radius of chassis motors, (mm).
     * @param[in] gearRatio Gear ratio of the chassis motors.
     * @param[in] railLength Length of the sentinel rail (mm).
     * @param[in] sentinelLength Length of the sentinel (mm).
     * @param[in] maxEnergyBuffer @see PowerLimiter
     * @param[in] energyBufferLimitThreshold @see PowerLimiter
     * @param[in] energyBufferCritThreshold @see PowerLimiter
     * @param[in] powerConsumptionThreshold @see PowerLimiter
     * @param[in] currentAllocatedForEnergyBufferLimiting @see PowerLimiter
     * @param[in] leftMotorId DJI motor id for left motor.
     * @param[in] rightMotorId DJI motor id for right motor.
     * @param[in] chassisCanBus CAN bus chassis is connected to.
     */
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
        float railLength,
        float sentinelLength,
        float maxEnergyBuffer,
        float energyBufferLimitThreshold,
        float energyBufferCritThreshold,
        float powerConsumptionThreshold,
        float currentAllocatedForEnergyBufferLimiting,
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
    const float RAIL_LENGTH;
    const float SENTINEL_LENGTH;

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

    aruwlib::control::chassis::PowerLimiter powerLimiter;
};

}  // namespace aruwsrc::control::sentinel::drive

#endif
