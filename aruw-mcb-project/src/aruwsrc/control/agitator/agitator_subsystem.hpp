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

#include "aruwlib/architecture/conditional_timer.hpp"
#include "aruwlib/architecture/timeout.hpp"
#include "aruwlib/control/subsystem.hpp"
#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
#include "aruwlib/mock/dji_motor_mock.hpp"
#else
#include "aruwlib/motor/dji_motor.hpp"
#endif

#include "aruwlib/algorithms/smooth_pid.hpp"
#include "aruwlib/control/setpoint/algorithms/setpoint_continuous_jam_checker.hpp"
#include "aruwlib/control/setpoint/interfaces/setpoint_subsystem.hpp"
#include "aruwlib/util_macros.hpp"

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
class AgitatorSubsystem : public aruwlib::control::setpoint::SetpointSubsystem
{
public:
    /**
     * @brief Construct an agitator with the passed in PID parameters, gear ratio, and
     * motor-specific identifiers.
     *
     * @param[in] drivers Pointer to a drivers singleton object.
     * @param[in] pidConfig PID configuration for position controller.
     * @param[in] agitatorGearRatio Gear ratio of agitator motor.
     * @param[in] agitatorMotorId DJI motor id of agitator motor.
     * @param[in] agitatorCanBusId CAN bus that the agitator motor is on.
     * @param[in] isAgitatorInverted Whether or not the agitator is inverted.
     * @param[in] jamLogicEnabled Whether or not jam logic is enabled. If `false`,
     *      `isJammed` always returns `false`.
     * @param[in] jamDistanceTolerance @see SetpointContinuousJamChecker.
     * @param[in] jamTemporalTolerance @see SetpointContinuousJamChecker.
     */
    AgitatorSubsystem(
        aruwlib::Drivers* drivers,
        const aruwlib::algorithms::PidConfigStruct& pidConfig,
        float agitatorGearRatio,
        aruwlib::motor::MotorId agitatorMotorId,
        aruwlib::can::CanBus agitatorCanBusId,
        bool isAgitatorInverted,
        bool jamLogicEnabled,
        float jamDistanceTolerance,
        uint32_t jamTemporalTolerance);

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
    aruwlib::algorithms::SmoothPid agitatorPositionPid;

    /**
     * The object that runs jam detection.
     */
    aruwlib::control::setpoint::SetpointContinuousJamChecker jamChecker;

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
    aruwlib::mock::DjiMotorMock agitatorMotor;

private:
#else
    aruwlib::motor::DjiMotor agitatorMotor;
#endif
};  // class AgitatorSubsystem

}  // namespace agitator

}  // namespace aruwsrc

#endif  // AGITATOR_SUBSYSTEM_HPP_
