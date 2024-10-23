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

#ifndef SENTRY_DRIVE_SUBSYSTEM_HPP_
#define SENTRY_DRIVE_SUBSYSTEM_HPP_

#include "tap/communication/gpio/digital.hpp"
#include "tap/control/chassis/chassis_subsystem_interface.hpp"

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
#include "tap/mock/dji_motor_mock.hpp"
#else
#include "tap/motor/dji_motor.hpp"
#endif

#include "tap/communication/sensors/current/analog_current_sensor.hpp"
#include "tap/control/chassis/power_limiter.hpp"
#include "tap/motor/m3508_constants.hpp"
#include "tap/util_macros.hpp"

#include "modm/math/filter/pid.hpp"

namespace aruwsrc::control::sentry::drive
{
class SentryDriveSubsystem : public tap::control::chassis::ChassisSubsystemInterface
{
public:
    /// @see power_limiter.hpp for what these mean
    static constexpr float STARTING_ENERGY_BUFFER = 200.0f;
    static constexpr float ENERGY_BUFFER_LIMIT_THRESHOLD = 100.0f;
    static constexpr float ENERGY_BUFFER_CRIT_THRESHOLD = 10;

    // radius of the wheel in mm
    static constexpr float WHEEL_RADIUS = 44.45f;
    static constexpr float SENTRY_LENGTH = 600.0f;

    static constexpr float GEAR_RATIO = 3591.0f / 187.0f;

    // RMUL length of the rail, in mm
    static constexpr float RAIL_LENGTH = 2130;
    // Our length of the rail, in mm
    // static constexpr float RAIL_LENGTH = 1900;

    /// Distance from either end of the rail (in mm) at which the sentry will cut its speed
    /// to avoid ramming into the wall.
    static constexpr float SPEED_REDUCTION_RAIL_BUFFER = 600.0f;
    /// Lowest allowable chassis speed scaling when close to the sides of the rail.
    static constexpr float MINIMUM_SENTRY_SPEED_MULTIPLIER = 0.1f;

    SentryDriveSubsystem(
        tap::Drivers* drivers,
        tap::gpio::Digital::InputPin leftLimitSwitch,
        tap::gpio::Digital::InputPin rightLimitSwitch,
        tap::motor::MotorId leftMotorId = LEFT_MOTOR_ID,
        tap::gpio::Analog::Pin currentSensorPin = CURRENT_SENSOR_PIN);

    void initialize() override;

    /**
     * Returns absolute position (in mm) of the sentry, relative to the left end of the rail (when
     * rail is viewed from the front)
     */
    mockable float getAbsolutePosition() const;

    mockable void setDesiredRpm(float desRpm);

    mockable float getDesiredRpm();

    void refresh() override;

    const char* getName() const override { return "Sentry Drive"; }

    inline int getNumChassisMotors() const override { return NUM_CHASSIS_MOTORS; }

    inline bool allMotorsOnline() const override
    {
        bool allOnline = true;
        for (int i = 0; i < NUM_CHASSIS_MOTORS; i++)
        {
            allOnline &= chassisMotors[i]->isMotorOnline();
        }
        return allOnline;
    }

    /**
     * @return The actual chassis velocity in chassis relative frame, as a vector <vx, vy, vz>,
     *      where vz is rotational velocity. Since the sentry is constrained to a single
     *      axis, vx and vz are 0. This is the velocity calculated from the chassis's
     *      encoders. Units: m/s
     */
    modm::Matrix<float, 3, 1> getActualVelocityChassisRelative() const override;

    /**
     * @param[in] currentPosition The current position of the sentry chassis, in millimeters.
     * @param[in] nearStartOfRailBuffer The distance in millimeters from the start of the rail
     * by which the sentry will be considered to be near the start of the rail.
     * @return true if the sentry is near the start of the rail (as indicated by the
     * `currentPosition`).
     */
    static inline bool nearStartOfRail(float currentPosition, float nearStartOfRailBuffer)
    {
        return currentPosition <= nearStartOfRailBuffer;
    }

    /**
     * @param[in] currentPosition The current position of the sentry chassis, in millimeters.
     * @param[in] nearEndOfRailBuffer The distance in millimeters from the start of the rail
     * by which the sentry will be considered to be near the end of the rail.
     * @return true if the sentry is near the end of the rail (as indicated by the
     * `currentPosition`).
     */
    static inline bool nearEndOfRail(float currentPosition, float nearEndOfRailBuffer)
    {
        float railEndPositionWithTurnaroundBuffer = SentryDriveSubsystem::RAIL_LENGTH -
                                                    SentryDriveSubsystem::SENTRY_LENGTH -
                                                    nearEndOfRailBuffer;

        return currentPosition >= railEndPositionWithTurnaroundBuffer;
    }

private:
    static constexpr tap::motor::MotorId LEFT_MOTOR_ID = tap::motor::MOTOR3;
    static constexpr tap::can::CanBus CAN_BUS_MOTORS = tap::can::CanBus::CAN_BUS2;
    static constexpr tap::gpio::Analog::Pin CURRENT_SENSOR_PIN = tap::gpio::Analog::Pin::S;

    static constexpr float PID_P = 5.0f;
    static constexpr float PID_I = 0.0f;
    static constexpr float PID_D = 0.1f;
    static constexpr float PID_MAX_ERROR_SUM = 0.0f;
    static constexpr float PID_MAX_OUTPUT = 10000;

    tap::gpio::Digital::InputPin leftLimitSwitch;
    tap::gpio::Digital::InputPin rightLimitSwitch;

    modm::Pid<float> velocityPidLeftWheel;

    modm::Pid<float> velocityPidRightWheel;

    float desiredRpm = 0;
    float leftWheelZeroRailOffset = 0;
    float rightWheelZeroRailOffset = 0;

    void resetOffsetFromLimitSwitch();

    float distanceFromEncoder(const tap::motor::DjiMotor* motor) const;

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
public:
    testing::NiceMock<tap::mock::DjiMotorMock> leftWheel;

private:
#else
    tap::motor::DjiMotor leftWheel;
#endif

    static constexpr int NUM_CHASSIS_MOTORS = 1;

    tap::motor::DjiMotor* chassisMotors[NUM_CHASSIS_MOTORS];

    tap::communication::sensors::current::AnalogCurrentSensor currentSensor;

    tap::control::chassis::PowerLimiter powerLimiter;

    /**
     * Computes a scalar that should be applied to the desiredRpm that is based on how close the
     * sentry is to the side of the rail. This scalar is applied to reduce the speed at which the
     * sentry runs into the side of the rail.
     *
     * @param[in] desiredRpm The desired wheel speed of the chassis motors in RPM.
     * @param[in] currentPosition The current position of the sentry on the rail, in mm.
     * @return A scalar between [0, 1] that the desiredRpm should be multiplied by.
     */
    static inline float computeEndOfRailSpeedReductionScalar(
        float desiredRpm,
        float currentPosition)
    {
        // Scalar between [0, 1] that we scale the desiredRpm by to slow down the sentry near the
        // ends of the rail
        float desiredRpmNearRailSidesScalar = 1.0f;

        currentPosition = tap::algorithms::limitVal<float>(currentPosition, 0, RAIL_LENGTH);

        if (nearStartOfRail(currentPosition, SPEED_REDUCTION_RAIL_BUFFER) && desiredRpm <= 0)
        {
            desiredRpmNearRailSidesScalar = currentPosition / SPEED_REDUCTION_RAIL_BUFFER;
        }
        else if (nearEndOfRail(currentPosition, SPEED_REDUCTION_RAIL_BUFFER) && desiredRpm >= 0)
        {
            desiredRpmNearRailSidesScalar =
                (RAIL_LENGTH - SENTRY_LENGTH - currentPosition) / SPEED_REDUCTION_RAIL_BUFFER;
        }

        desiredRpmNearRailSidesScalar =
            std::max(desiredRpmNearRailSidesScalar, MINIMUM_SENTRY_SPEED_MULTIPLIER);

        return desiredRpmNearRailSidesScalar;
    }
};

}  // namespace aruwsrc::control::sentry::drive

#endif  // SENTRY_DRIVE_SUBSYSTEM_HPP_
