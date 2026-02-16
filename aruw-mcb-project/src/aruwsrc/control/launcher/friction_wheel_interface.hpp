/*
 * Copyright (c) 2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef FRICTION_WHEEL_INTERFACE_HPP_
#define FRICTION_WHEEL_INTERFACE_HPP_

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/drivers.hpp"


namespace aruwsrc::control::launcher
{
/**
 * @brief Config struct for each flywheel in the friction wheel system. Contains the velocity PID
 * config for the flywheel, as well as the orientation of the flywheel and what stage of the
 * flywheel it is (currently unused beyond the velocity PID)
 */
struct FlywheelConfig
{
    tap::algorithms::SmoothPidConfig velocityPidConfig;
    float orientation;
    uint8_t stage = 0;
};

/**
 * A wrapper interface to hide the templates and allow FrictionWheelSubsystems to be easily passes
 * as parameters.
 */
class FrictionWheelInterface : public tap::control::Subsystem
{
public:
    /**
     * Basic constructor to initialize tap::control::Subsystem class.
     */
    FrictionWheelInterface(tap::Drivers* drivers) : tap::control::Subsystem(drivers) {}

    /**
     * @return The predicted launch speed of the next projectile in m/s, using measured feedback
     * from the referee system barrel system to dynamically predict the barrel speed based on
     * previous barrel speeds.
     */
    float getPredictedLaunchSpeed() const;

    /**
     * Set the projectile launch speed - at what speed the pellets
     * will come out of the physical friction wheel launcher.
     * Speed is limited to a range of values defined by constants of
     * this subsystem.
     *
     * @param[in] speed The launch speed in m/s.
     * @param[in] directRpm Whether to directly set rpm or set bullet speed.
     */
    virtual void setDesiredLaunchSpeed(float speed, bool directRpm = false) = 0;

    /**
     * Sets the target velocity for an individual friction wheel. Velocity is only used if wheel's
     * velocity state is also changed.
     *
     * @param[in] index The index of the friction wheel to set, starting at 0.
     * @param[in] velocity The velocity in rpm of the desired friction wheel.
     */
    virtual void setIndividualVelocity(int index, float velocity) = 0;

    /**
     * Sets the velocity state for an individual friction to be either desired launch speed or
     * specific rpm.
     *
     * @param[in] index The index of the friction wheel to set, starting at 0.
     * @param[in] hasIndividualVelocity True if the friction wheel is set to its own rpm, false if
     * using desired launch speed.
     */
    virtual void changeWheelVelocityState(int index, bool hasIndividualVelocity) = 0;

    /**
     * @return The desired launch speed in m/s, based on the setDesiredLaunchSpeed method.
     */
    virtual float getDesiredLaunchSpeed() const = 0;

    /**
     * @return The desired speed each friction wheel is set to in rpm, including speed correction.
     */
    virtual float getDesiredFrictionWheelSpeed() const = 0;

    /**
     * @return Speed correction for friction wheel rpm.
     */
    virtual float getCurrentCorrectionValue() const = 0;

    /**
     * @return The average measured friction wheel speed of the launcher in RPM.
     */
    virtual float getCurrentAverageFrictionWheelSpeed() const = 0;

    /**
     * @param[in] index Integer representing desired flywheel, starting at 0.
     * @return The measured friction wheel speed of the nth flywheel in launcher in RPM.
     */
    virtual float getCurrentIndividualFrictionWheelSpeed(int index) const = 0;

    virtual const char* getName() const = 0;
};
}  // namespace aruwsrc::control::launcher
#endif