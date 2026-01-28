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

#include "aruwsrc/control/control_operator_interface.hpp"

namespace aruwsrc::engineer
{
class EngineerControlOperatorInterface : public control::ControlOperatorInterface
{
public:
    EngineerControlOperatorInterface(tap::Drivers *drivers)
        : ControlOperatorInterface(drivers),
          drivers(drivers)
    {
    }

    virtual float getChassisXInput() override;

    virtual float getChassisYInput() override;

    virtual float getChassisRInput() override;

    /**
     * @return The cube storage velocity.
     */
    mockable float getCubeStorageVelocity();

    /**
     * @return The gantry lift velocity
     */
    mockable float getGantryLiftVelocity();

    bool getGantryKeyUp();

    bool getGantryKeyDown();

    bool getGantryKeyIn();

    bool getGantryKeyOut();

    bool getSprintKey();

    bool getShiftKey();

    /**
     * @return The gantry extension velocity
     */
    mockable float getGantryExtensionVelocity();

    /**
     * @return The wrist pitch velocity
     */
    mockable float getWristPitchVelocity();

    /**
     * @return The wrist yaw velocity
     */
    mockable float getWristYawVelocity();

    /**
     * @return The wrist roll velocity
     */
    mockable float getWristRollVelocity();

    /**
     * @return whether or not the control switch is set to drive mode.
     */
    bool isDriveMode();

    /**
     * @return whether or not the control switch is set to gantry (lift + horizontal extension)
     * control mode.
     */
    bool isGantryWristControlMode();

     /**
     * @return whether or not the control switch is set to cube storage control mode.
     */
    bool isCubeStorageControlMode();

private:
    tap::Drivers *drivers;
    float divideValPitch = 375.0f;  // these are all scaling factors for driver control
    float divideValYaw = 375.0f;
    float divideGantryLift = 375.0f;
    float divideGantryExtension = 375.0f;
};
}  // namespace aruwsrc::engineer

#endif  // ENGINEER_CONTROL_OPERATOR_INTERFACE_HPP__