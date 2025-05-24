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

#ifndef ENGINEER_CONTROL_OPERATOR_INTERFACE_HPP_
#define ENGINEER_CONTROL_OPERATOR_INTERFACE_HPP_

#include "tap/algorithms/linear_interpolation_predictor.hpp"
#include "tap/algorithms/ramp.hpp"
#include "tap/architecture/clock.hpp"
#include "tap/drivers.hpp"

#include "aruwsrc/robot/control_operator_interface.hpp"

namespace aruwsrc::control::engineer
{
class EngineerControlOperatorInterface : public ControlOperatorInterface
{
public:
    EngineerControlOperatorInterface(tap::Drivers *drivers)
        : ControlOperatorInterface(drivers),
          drivers(drivers)
    {
    }

    virtual float getChassisXInput() override;

    virtual float getChassisYInput() override;

    /**
     * @return The cube lift velocity.
     */
    mockable float getCubeLiftVelocity();

    /**
     * @return The arm lift velocity
     */
    mockable float getArmLiftVelocity();

    /**
     * @return The arm horizontal stage velocity
     */
    mockable float getArmExtensionVelocity();

    /**
     * @return The arm wrist pitch velocity
     */
    mockable float getArmWristPitchVelocity();

    /**
     * @return The arm wrist yaw velocity
     */
    mockable float getArmWristYawVelocity();

    /**
     * @return The arm wrist roll velocity
     */
    mockable float getArmWristRollVelocity();

    /**
     * @return whether or not the control switch is set to drive mode.
     */
    bool isDriveMode();

    /**
     * @return whether or not the control switch is set to gantry (lift + horizontal extension)
     * control mode.
     */
    bool isGantryControlMode();

    /**
     * @return whether or not the control switch is set to wrist control mode.
     */
    bool isWristControlMode();

private:
    tap::Drivers *drivers;
};
}  // namespace aruwsrc::control::engineer

#endif  // ENGINEER_CONTROL_OPERATOR_INTERFACE_HPP__