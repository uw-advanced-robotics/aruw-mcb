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

#include "tap/drivers.hpp"
#include "tap/algorithms/ramp.hpp"
#include "tap/architecture/clock.hpp"


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

    tap::algorithms::Ramp chassisXInputRamp;
    tap::algorithms::Ramp chassisYInputRamp;
    tap::algorithms::Ramp chassisYawInputRamp;

    uint32_t prevChassisXInputCalledTime = 0;
    uint32_t prevChassisYInputCalledTime = 0;
    uint32_t prevChassisYawInputCalledTime = 0;

    uint32_t getDT(uint32_t prevTimeCalled);

};

} // namespace aruwsrc::control::sentry
