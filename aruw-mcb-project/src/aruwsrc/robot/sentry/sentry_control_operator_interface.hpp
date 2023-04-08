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

#ifndef CONTROL_OPERATOR_INTERFACE_HPP_
#define CONTROL_OPERATOR_INTERFACE_HPP_


#include "tap/drivers.hpp"
#include "tap/algorithms/ramp.hpp"
#include "tap/architecture/clock.hpp"
#include "tap/algorithms/linear_interpolation_predictor.hpp"
#include "src/aruwsrc/robot/sentry/sentry_knuckles_turret_constants.hpp"

namespace aruwsrc::control::sentry
{

class SentryControlOperatorInterface
{
public:
    SentryControlOperatorInterface(tap::Drivers *drivers) : drivers(drivers) {}

    /**
     * @return The value used for chassis movement forward and backward, between
     * `[-getMaxUserWheelSpeed, getMaxUserWheelSpeed]`. Acceleration is applied to this value
     * controlled by `MAX_ACCELERATION_X` and `MAX_DECELERATION_X`. A linear combination of keyboard
     * and remote joystick information.
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

    /**
    * Updates the control operator's state ??
    */
    mockable float update();

    enum class ControllerMode {
        AUTO=0,
        SHOOTING,
        DRIVING,
        NUM_MODES
    };

private:
    tap::Drivers *drivers;

    uint32_t prevUpdateCounterChassisXInput = 0;
    uint32_t prevUpdateCounterChassisYInput = 0;
    uint32_t prevUpdateCounterTurretMajorYawInput = 0;
    uint32_t prevUpdateCounterTurretMinor1YawInput = 0;
    uint32_t prevUpdateCounterTurretMinor2YawInput = 0;
    uint32_t prevUpdateCounterTurretMinor1PitchInput = 0;
    uint32_t prevUpdateCounterTurretMinor2PitchInput = 0;

    tap::algorithms::LinearInterpolationPredictor chassisXInput;
    tap::algorithms::LinearInterpolationPredictor chassisYInput;
    tap::algorithms::LinearInterpolationPredictor turretMajorYawInput;
    tap::algorithms::LinearInterpolationPredictor turretMinor1YawInput;
    tap::algorithms::LinearInterpolationPredictor turretMinor2YawInput;
    tap::algorithms::LinearInterpolationPredictor turretMinor1PitchInput;
    tap::algorithms::LinearInterpolationPredictor turretMinor2PitchInput;

    tap::algorithms::Ramp chassisXInputRamp;
    tap::algorithms::Ramp chassisYInputRamp;
    tap::algorithms::Ramp turretMajorYawRamp;
    tap::algorithms::Ramp turretMinor1YawRamp;
    tap::algorithms::Ramp turretMinor2YawRamp;
    tap::algorithms::Ramp turretMinor1PitchRamp;
    tap::algorithms::Ramp turretMinor2PitchRamp;

    uint32_t prevChassisXInputCalledTime = 0;
    uint32_t prevChassisYInputCalledTime = 0;
    uint32_t prevTurretMajorYawInputCalledTime = 0;
    uint32_t prevTurretMinor1YawInputCalledTime = 0;
    uint32_t prevTurretMinor2YawInputCalledTime = 0;
    uint32_t prevTurretMinor1PitchInputCalledTime = 0;
    uint32_t prevTurretMinor2PitchInputCalledTime = 0;
};

} // namespace aruwsrc::control::sentry


#endif // CONTROL_OPERATOR_INTERFACE_HPP_