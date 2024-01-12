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

#ifndef SENTRY_CONTROL_OPERATOR_INTERFACE_HPP_
#define SENTRY_CONTROL_OPERATOR_INTERFACE_HPP_

#include "tap/algorithms/linear_interpolation_predictor.hpp"
#include "tap/algorithms/ramp.hpp"
#include "tap/architecture/clock.hpp"
#include "tap/drivers.hpp"

#include "aruwsrc/control/turret/constants/turret_constants.hpp"

namespace aruwsrc::control::sentry
{
class SentryControlOperatorInterface
{
public:
    float lastWheelVal = 0.0f;
    float DEFAULT_CHASSIS_X_VELOCITY = 0.f;
    float DEFAULT_CHASSIS_Y_VELOCITY = 0.f;
    float DEFAULT_CHASSIS_YAW_VELOCITY = 0.f;
    float DEFAULT_TURRET_MAJOR_VELOCITY = 0.f;

    /**
     * Max acceleration in rpm/s^2 of the chassis in the x direction
     */
    static constexpr float MAX_ACCELERATION_X = 10'000.0f;  // TODO: change these values
    static constexpr float MAX_DECELERATION_X = 20'000.0f;
    static constexpr float MAX_ACCELERATION_Y = 10'000.0f;  // TODO: change the values?
    static constexpr float MAX_DECELERATION_Y = 20'000.0f;
    static constexpr float MAX_ACCELERATION_R = 10'000.0f;  // TODO: change the values?
    static constexpr float MAX_DECELERATION_R = 20'000.0f;
    static constexpr float MAX_CHASSIS_YAW_SPEED = 200;         // TODO: refine this
    static constexpr float MAX_TURRET1_MINOR_YAW_SPEED = 10;    // TODO: refine this
    static constexpr float MAX_TURRET2_MINOR_YAW_SPEED = 10;    // TODO: refine this
    static constexpr float MAX_TURRET1_MINOR_PITCH_SPEED = 10;  // TODO: refine this
    static constexpr float MAX_TURRET2_MINOR_PITCH_SPEED = 10;  // TODO: refine this

    SentryControlOperatorInterface(tap::Drivers *drivers) : drivers(drivers) {}

    // Drive mode functions

    /**
     * @return The value used for chassis movement forward and backward
     */
    mockable float getChassisXVelocity();

    /**
     * @return The value used for chassis lateral movement
     */
    mockable float getChassisYVelocity();

    /**
     * @return The value used for chassis rotational velocity,
     */
    mockable float getChassisYawVelocity();

    // Turret control mode functions
    /**
     * @return the value used for turret major yaw velocity in radians / second
     */
    mockable float getTurretMajorYawVelocity();

    /**
     * @return the value used for turret minor 1 yaw velocity in radians / second
     */
    mockable float getTurretMinor1YawVelocity();

    /**
     * @return the value used for turret minor 1 pitch velocity in radians / second
     */
    mockable float getTurretMinor1PitchVelocity();

    /**
     * @return the value used for turret minor 2 yaw velocity in radians / second
     */
    mockable float getTurretMinor2YawVelocity();

    /**
     * @return the value used for turret minor 2 pitch velocity in radians / second
     */
    mockable float getTurretMinor2PitchVelocity();

    // TODO: add autodrive commands

    /**
     * @return whether or not the control switch is set to drive mode.
     */
    bool isDriveMode();

    /**
     * @return whether or not the control switch is set to turret control mode.
     */
    bool isTurretControlMode();

    /**
     * @return whether or not the control switch is set to auto drive mode.
     */
    bool isAutoDriveMode();

private:
    tap::Drivers *drivers;

    uint32_t prevUpdateCounterChassisXInput = 0;
    uint32_t prevUpdateCounterChassisYInput = 0;
    uint32_t prevUpdateCounterChassisYawInput = 0;

    uint32_t prevUpdateCounterTurretMajorYawInput = 0;
    uint32_t prevUpdateCounterTurretMinor1YawInput = 0;
    uint32_t prevUpdateCounterTurretMinor2YawInput = 0;
    uint32_t prevUpdateCounterTurretMinor1PitchInput = 0;
    uint32_t prevUpdateCounterTurretMinor2PitchInput = 0;

    tap::algorithms::LinearInterpolationPredictor chassisXInput;
    tap::algorithms::LinearInterpolationPredictor chassisYInput;
    tap::algorithms::LinearInterpolationPredictor chassisYawInput;
    tap::algorithms::LinearInterpolationPredictor turretMajorYawInput;
    tap::algorithms::LinearInterpolationPredictor turretMinor1YawInput;
    tap::algorithms::LinearInterpolationPredictor turretMinor2YawInput;
    tap::algorithms::LinearInterpolationPredictor turretMinor1PitchInput;
    tap::algorithms::LinearInterpolationPredictor turretMinor2PitchInput;

    tap::algorithms::Ramp chassisXInputRamp;
    tap::algorithms::Ramp chassisYInputRamp;
    tap::algorithms::Ramp chassisYawInputRamp;
    tap::algorithms::Ramp turretMajorYawRamp;
    tap::algorithms::Ramp turretMinor1YawRamp;
    tap::algorithms::Ramp turretMinor2YawRamp;
    tap::algorithms::Ramp turretMinor1PitchRamp;
    tap::algorithms::Ramp turretMinor2PitchRamp;

    uint32_t prevChassisXInputCalledTime = 0;
    uint32_t prevChassisYInputCalledTime = 0;
    uint32_t prevChassisYawnputCalledTime = 0;
    uint32_t prevTurretMajorYawInputCalledTime = 0;
    uint32_t prevTurretMinor1YawInputCalledTime = 0;
    uint32_t prevTurretMinor2YawInputCalledTime = 0;
    uint32_t prevTurretMinor1PitchInputCalledTime = 0;
    uint32_t prevTurretMinor2PitchInputCalledTime = 0;
};

}  // namespace aruwsrc::control::sentry

#endif  // SENTRY_CONTROL_OPERATOR_INTERFACE_HPP__
