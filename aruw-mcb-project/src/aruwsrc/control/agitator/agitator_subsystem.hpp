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

#ifndef AGITATOR_SUBSYSTEM_HPP_
#define AGITATOR_SUBSYSTEM_HPP_

#include "tap/architecture/conditional_timer.hpp"
#include "tap/architecture/timeout.hpp"
#include "tap/control/subsystem.hpp"
#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
#include "tap/mock/dji_motor_mock.hpp"
#else
#include "tap/motor/dji_motor.hpp"
#endif

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/control/setpoint/algorithms/setpoint_continuous_jam_checker.hpp"
#include "tap/control/setpoint/interfaces/setpoint_subsystem.hpp"
#include "tap/util_macros.hpp"

#include "aruwsrc/util_macros.hpp"

namespace aruwsrc
{
class Drivers;
}

namespace aruwsrc
{
namespace agitator
{
/**
 * Subsystem whose primary purpose is to encapsulate an agitator motor
 * that operates using a position controller. While this subsystem provides
 * direct support for agitator control, it is generic enough to be used in a
 * wide variety of senarios.
 */
class AgitatorSubsystem : public tap::control::setpoint::SetpointSubsystem
{
public:
#if defined(ALL_SOLDIERS)
    // position PID terms
    // PID terms for soldier
    static constexpr float PID_17MM_P = 300'000.0f;
    static constexpr float PID_HOPPER_P = 100000.0f;
    static constexpr float PID_17MM_I = 0.0f;
    static constexpr float PID_17MM_D = 50.0f;
    static constexpr float PID_17MM_MAX_ERR_SUM = 0.0f;
    static constexpr float PID_17MM_MAX_OUT = 16000.0f;

    static constexpr tap::motor::MotorId AGITATOR_MOTOR_ID = tap::motor::MOTOR7;
    static constexpr tap::can::CanBus AGITATOR_MOTOR_CAN_BUS = tap::can::CanBus::CAN_BUS1;

    static constexpr bool isAgitatorInverted = false;

    /**
     * The jamming constants. Agitator is considered jammed if difference between setpoint
     * and current angle is > `JAMMING_DISTANCE` radians for >= `JAMMING_TIME` ms;
     *
     * @warning: `JAMMING_DISTANCE` must be less than the smallest movement command
     *
     * This should be positive or else weird behavior can occur
     */
    static constexpr float AGITATOR_JAMMING_DISTANCE = M_PI / 20;
    static constexpr uint32_t JAMMING_TIME = 250;

    // The motor that controls the hopper lid is an agitator_subsystem instance, so
    // I'm adding its constants here as well.
    static constexpr tap::motor::MotorId HOPPER_COVER_MOTOR_ID = tap::motor::MOTOR8;
    static constexpr tap::can::CanBus HOPPER_COVER_MOTOR_CAN_BUS = tap::can::CanBus::CAN_BUS1;

    static constexpr bool IS_HOPPER_COVER_INVERTED = false;

#elif defined(TARGET_SENTINEL)
    // position PID terms
    // PID terms for sentinel
    static constexpr float PID_17MM_P = 300'000.0f;
    static constexpr float PID_17MM_I = 0.0f;
    static constexpr float PID_17MM_D = 50.0f;
    static constexpr float PID_17MM_MAX_ERR_SUM = 0.0f;
    static constexpr float PID_17MM_MAX_OUT = 16000.0f;

    static constexpr tap::motor::MotorId AGITATOR_MOTOR_ID = tap::motor::MOTOR7;
    static constexpr tap::can::CanBus AGITATOR_MOTOR_CAN_BUS = tap::can::CanBus::CAN_BUS1;

#elif defined(TARGET_HERO)
    // Hero's waterwheel constants
    static constexpr float PID_HERO_WATERWHEEL_P = 150'000.0f;
    static constexpr float PID_HERO_WATERWHEEL_I = 0.0f;
    static constexpr float PID_HERO_WATERWHEEL_D = 50.0f;
    static constexpr float PID_HERO_WATERWHEEL_MAX_ERR_SUM = 0.0f;
    static constexpr float PID_HERO_WATERWHEEL_MAX_OUT = 16000.0f;

    static constexpr tap::motor::MotorId HERO_WATERWHEEL_MOTOR_ID = tap::motor::MOTOR4;
    static constexpr tap::can::CanBus HERO_WATERWHEEL_MOTOR_CAN_BUS = tap::can::CanBus::CAN_BUS1;
    static constexpr bool HERO_WATERWHEEL_INVERTED = false;

    // PID terms for the hero kicker
    static constexpr float PID_HERO_KICKER_P = 100'000.0f;
    static constexpr float PID_HERO_KICKER_I = 0.0f;
    static constexpr float PID_HERO_KICKER_D = 50.0f;
    static constexpr float PID_HERO_KICKER_MAX_ERR_SUM = 0.0f;
    // max out added by Tenzin since it wasn't here. This should
    // also be changed by someone who know's what they're doing!
    static constexpr float PID_HERO_KICKER_MAX_OUT = 16000.0f;

    static constexpr tap::motor::MotorId HERO_KICKER_MOTOR_ID = tap::motor::MOTOR8;
    static constexpr tap::can::CanBus HERO_KICKER_MOTOR_CAN_BUS = tap::can::CanBus::CAN_BUS1;
    static constexpr bool HERO_KICKER_INVERTED = false;

    /**
     * The jamming constants for waterwheel. Waterwheel is considered jammed if difference between
     * setpoint and current angle is > `JAM_DISTANCE_TOLERANCE_WATERWHEEL` radians for >=
     * `JAM_TEMPORAL_TOLERANCE_WATERWHEEL` ms;
     */
    static constexpr float JAM_DISTANCE_TOLERANCE_WATERWHEEL = M_PI / 14.0f;
    static constexpr uint32_t JAM_TEMPORAL_TOLERANCE_WATERWHEEL = 100.0f;
#endif

    /**
     * Agitator gear ratios of different motors, for determining shaft rotation angle.
     */
    static constexpr float AGITATOR_GEAR_RATIO_M2006 = 36.0f;
    static constexpr float AGITATOR_GEAR_RATIO_GM3508 = 19.0f;

    /**
     * Construct an agitator with the passed in PID parameters, gear ratio, and motor-specific
     * identifiers.
     *
     * Jam parameters are not used if jam logic is disabled.
     *
     * @param[in] drivers pointer to aruwsrc drivers struct
     * @param[in] kp PID kp constant
     * @param[in] ki PID ki constant
     * @param[in] kd PID kd constant
     * @param[in] maxIAccum limit on integral value in PID
     * @param[in] maxOutput max output of PID
     * @param[in] agitatorGearRatio the gear ratio of this motor
     * @param[in] agitatorMotorId the motor ID for this motor
     * @param[in] isAgitatorInverted if `true` positive rotation is clockwise when
     *      looking at the motor shaft opposite the motor. Counterclockwise if false
     * @param[in] jammingDistance jamming timer counts down when distance between
     *      setpoint and current angle is > `jammingDistance` and resets timer when
     *      distance is <= `jammingDistance`.
     * @param[in] jammingTime how long the jamming timer is. Once this timer finishes
     *      the subsystem is considered jammed
     * @param[in] jamLogicEnabled whether or not to enable jam detection
     */
    AgitatorSubsystem(
        aruwsrc::Drivers* drivers,
        float kp,
        float ki,
        float kd,
        float maxIAccum,
        float maxOutput,
        float agitatorGearRatio,
        tap::motor::MotorId agitatorMotorId,
        tap::can::CanBus agitatorCanBusId,
        bool isAgitatorInverted,
        float jammingDistance,
        uint32_t jammingTime,
        bool jamLogicEnabled);

    void initialize() override;

    void refresh() override;

    /**
     * @return The angle set in `setSetpoint`.
     */
    mockable inline float getSetpoint() const override { return desiredAgitatorAngle; }

    /**
     * Sets desired angle in radians of the agitator motor, relative to where the agitator
     * has been initialized.
     *
     * @param[in] newAngle The desired angle.
     */
    mockable inline void setSetpoint(float newAngle) override { desiredAgitatorAngle = newAngle; }

    /**
     * @return The calibrated agitator angle, in radians. If the agitator is uncalibrated, 0
     *      radians is returned.
     */
    mockable float getCurrentValue() const override;

    /**
     * @return the setpoint tolerance. Returns the maximum distance in radians at which jam
     *      condition will never be triggered.
     */
    float getJamSetpointTolerance() const override;

    /**
     * Attempts to calibrate the agitator at the current position, such that
     * `getCurrentValue` will return 0 radians at this position.
     *
     * @return `true` if the agitator has been successfully calibrated, `false`
     *      otherwise.
     */
    mockable bool calibrateHere() override;

    /**
     * @return `true` if the agitator unjam timer has expired, signaling that the agitator
     *      has jammed, `false` otherwise.
     */
    mockable bool isJammed() override { return jamLogicEnabled && subsystemJamStatus; }

    /**
     * Clear the jam status of the subsystem, indicating that it has been unjammed.
     */
    void clearJam() override { subsystemJamStatus = false; }

    /**
     * @return `true` if the agitator has been calibrated (`calibrateHere` has been
     *      called and the agitator motor is online).
     */
    mockable inline bool isCalibrated() override { return agitatorIsCalibrated; }

    /**
     * @return `true` if the agitator motor is online (i.e.: is connected)
     */
    mockable inline bool isOnline() override { return agitatorMotor.isMotorOnline(); }

    /**
     * @return The velocity of the agitator in units of degrees per second.
     */
    mockable inline float getVelocity() override
    {
        return 6.0f * static_cast<float>(agitatorMotor.getShaftRPM()) / gearRatio;
    }

    void runHardwareTests() override;

    void onHardwareTestStart() override;

    mockable const char* getName() override { return "Agitator"; }

protected:
    /**
     * Whether or not the agitator has been calibrated yet. You should calibrate the agitator
     * before using it.
     */
    bool agitatorIsCalibrated = false;

    void agitatorRunPositionPid();

private:
    /**
     * PID controller for running postiion PID on unwrapped agitator angle (in radians).
     */
    tap::algorithms::SmoothPid agitatorPositionPid;

    /**
     * The object that runs jam detection.
     */
    tap::control::setpoint::SetpointContinuousJamChecker jamChecker;

    /**
     * The user desired angle, measured in radians.
     * The agitator uses unwrapped angle.
     */
    float desiredAgitatorAngle = 0.0f;

    /**
     * You can calibrate the agitator, which will set the current agitator angle to zero radians.
     */
    float agitatorCalibratedZeroAngle = 0.0f;

    /**
     * Motor gear ratio, so we use shaft angle rather than encoder angle.
     */
    float gearRatio;

    /**
     * Stores the jam state of the subsystem
     */
    bool subsystemJamStatus = false;

    /**
     * A flag which determines whether or not jamming detection is enabled.
     * `true` means enabled, `false` means disabled.
     * Detailed effect: When `false`, isJammed() always return false.
     */
    bool jamLogicEnabled;

    /**
     * Get the raw angle of the shaft from the motor
     */
    float getUncalibratedAgitatorAngle() const;

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
public:
    tap::mock::DjiMotorMock agitatorMotor;

private:
#else
    tap::motor::DjiMotor agitatorMotor;
#endif
};  // class AgitatorSubsystem

}  // namespace agitator

}  // namespace aruwsrc

#endif  // AGITATOR_SUBSYSTEM_HPP_
