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
     * @param[in] pidParams Position PID configuration struct for the agitator motor controller.
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
        tap::Drivers* drivers,
        const tap::algorithms::SmoothPidConfig& pidParams,
        float agitatorGearRatio,
        tap::motor::MotorId agitatorMotorId,
        tap::can::CanBus agitatorCanBusId,
        bool isAgitatorInverted,
        float jammingDistance,
        uint32_t jammingTime,
        bool jamLogicEnabled);

    void initialize() override;

    void refresh() override;

    void refreshSafeDisconnect() override { agitatorMotor.setDesiredOutput(0); }

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
    void clearJam() override
    {
        subsystemJamStatus = false;
        jamChecker.restart();
    }

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
    testing::NiceMock<tap::mock::DjiMotorMock> agitatorMotor;

private:
#else
    tap::motor::DjiMotor agitatorMotor;
#endif
};  // class AgitatorSubsystem

}  // namespace agitator

}  // namespace aruwsrc

#endif  // AGITATOR_SUBSYSTEM_HPP_
