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

#ifndef VELOCITY_AGITATOR_SUBSYSTEM
#define VELOCITY_AGITATOR_SUBSYSTEM

#include "tap/architecture/conditional_timer.hpp"
#include "tap/architecture/timeout.hpp"
#include "tap/control/subsystem.hpp"
#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
#include "tap/mock/dji_motor_mock.hpp"
#else
#include "tap/motor/dji_motor.hpp"
#endif

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/control/velocity/algorithms/velocity_continuous_jam_checker.hpp"
#include "tap/control/velocity/interfaces/velocity_setpoint_subsystem.hpp"
#include "tap/util_macros.hpp"

#include "aruwsrc/util_macros.hpp"

#include "velocity_agitator_subsystem_config.hpp"

namespace aruwsrc
{
class Drivers;
}

namespace aruwsrc::agitator
{
/**
 * Subsystem whose primary purpose is to encapsulate an agitator motor that operates using a
 * velocity controller. Also keeps track of absolute position to allow commands to rotate the
 * agitator some specific displacement.
 */
class VelocityAgitatorSubsystem : public tap::control::velocity::VelocitySetpointSubsystem
{
public:
    /**
     * Agitator gear ratios of different motors, for determining shaft rotation angle.
     */
    static constexpr float AGITATOR_GEAR_RATIO_M2006 = 36.0f;
    static constexpr float AGITATOR_GEAR_RATIO_GM3508 = 19.0f;

    /**
     * Construct an agitator with the passed in velocity PID parameters, gear ratio, and
     * agitator-specific configuration.
     *
     * @param[in] drivers pointer to aruwsrc drivers struct
     * @param[in] pidParams Position PID configuration struct for the agitator motor controller.
     * @param[in] agitatorSubsystemConfig Agitator configuration struct that contains
     * agitator-specific parameters including motor ID and unjam parameters.
     */
    VelocityAgitatorSubsystem(
        aruwsrc::Drivers* drivers,
        const tap::algorithms::SmoothPidConfig& pidParams,
        const VelocityAgitatorSubsystemConfig& agitatorSubsystemConfig);

    void initialize() override;

    void refresh() override;

    /// @return The velocity setpoint that some command has requested, in radians / second
    inline float getVelocitySetpoint() const override { return velocitySetpoint; }

    /**
     * Sets the velocity setpoint to the specified velocity
     *
     * @param[in] velocity The desired velocity in radians / second.
     */
    void setVelocitySetpoint(float velocity) override;

    /// @return The agitator velocity in radians / second.
    mockable inline float getVelocity() override
    {
        return (agitatorMotor.getShaftRPM() / config.gearRatio) * (M_TWOPI / 60.0f);
    }

    /**
     * @return The calibrated agitator angle, in radians. If the agitator is uncalibrated, 0
     * radians is returned.
     */
    mockable float getPosition() const override;

    /**
     * @return the setpoint tolerance. Returns the maximum distance in radians at which jam
     *      condition will never be triggered.
     */
    float getJamSetpointTolerance() const override;

    /**
     * Attempts to calibrate the agitator at the current position, such that `getPosition` will
     * return 0 radians at this position.
     *
     * @return `true` if the agitator has been successfully calibrated, `false` otherwise.
     */
    mockable bool calibrateHere() override;

    /**
     * @return `true` if the agitator unjam timer has expired, signaling that the agitator has
     * jammed, `false` otherwise.
     */
    mockable bool isJammed() override { return config.jamLogicEnabled && subsystemJamStatus; }

    /**
     * Clear the jam status of the subsystem, indicating that it has been unjammed.
     */
    inline void clearJam() override
    {
        subsystemJamStatus = false;
        jamChecker.restart();
    }

    /**
     * @return `true` if the agitator has been calibrated (`calibrateHere` has been called and the
     * agitator motor is online).
     */
    mockable inline bool isCalibrated() override { return agitatorIsCalibrated; }

    /**
     * @return `true` if the agitator motor is online (i.e.: is connected)
     */
    mockable inline bool isOnline() override { return agitatorMotor.isMotorOnline(); }

    void runHardwareTests() override;

    void onHardwareTestStart() override;

    mockable const char* getName() override { return "velocity agitator"; }

private:
    VelocityAgitatorSubsystemConfig config;

    tap::algorithms::SmoothPid velocityPid;

    /// The object that runs jam detection.
    tap::control::velocity::VelocityContinuousJamChecker jamChecker;

    /// You can calibrate the agitator, which will set the current agitator angle to zero radians.
    /// This value is the starting measured angle offset applied to make the motor angle "0" when
    /// `calibrateHere` is called.
    float agitatorCalibratedZeroAngle = 0.0f;

    /// Stores the jam state of the subsystem
    bool subsystemJamStatus = false;

    /**
     * Whether or not the agitator has been calibrated yet. You should calibrate the agitator
     * before using it.
     */
    bool agitatorIsCalibrated = false;

    /// Previous time the velocity controller was called, in milliseconds
    uint32_t prevTime = 0;

    /// The velocity setpoint in radians / second
    float velocitySetpoint = 0;

    /// Get the raw angle of the shaft from the motor, in radians
    float getUncalibratedAgitatorAngle() const;

    /// Runes the velocity PID controller
    void runVelocityPidControl();

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
public:
    testing::NiceMock<tap::mock::DjiMotorMock> agitatorMotor;

private:
#else
    tap::motor::DjiMotor agitatorMotor;
#endif
};

}  // namespace aruwsrc::agitator

#endif  // VELOCITY_AGITATOR_SUBSYSTEM
