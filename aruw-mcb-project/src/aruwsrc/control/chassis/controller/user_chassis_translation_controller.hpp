/*
 * Copyright (c) 2025-2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef USER_CHASSIS_TRANSLATION_CONTROLLER_HPP_
#define USER_CHASSIS_TRANSLATION_CONTROLLER_HPP_

#include "tap/algorithms/transforms/transform.hpp"

#include "aruwsrc/control/chassis/holonomic_chassis_subsystem.hpp"
#include "aruwsrc/control/control_operator_interface.hpp"

#include "frame_relative_chassis_translation_controller.hpp"

namespace aruwsrc::control::chassis::controller
{
/**
 * A common operation is to have the chassis translate in either turret or world frame (i.e. not
 * chassis frame). This class handles that transform.
 */
class UserChassisTranslationController : public FrameRelativeChassisTranslationController
{
public:
    UserChassisTranslationController(
        const tap::algorithms::transforms::Transform& chassisToTurret,
        const aruwsrc::control::chassis::HolonomicChassisSubsystem& chassis,
        aruwsrc::control::ControlOperatorInterface& operatorInterface)
        : FrameRelativeChassisTranslationController(chassisToTurret),
          operatorInterface(operatorInterface)
    {
    }

    tap::algorithms::transforms::Vector runFrameRelativeTranslationController(
        const float maxSpeed) final override
    {
        return tap::algorithms::transforms::Vector(
            operatorInterface.getChassisXInput(),
            operatorInterface.getChassisYInput(),
            0.0f);
    }

private:
    aruwsrc::control::ControlOperatorInterface& operatorInterface;
};  // class ChassisYawControllerInterface

}  // namespace aruwsrc::control::chassis::controller

#endif  // USER_CHASSIS_TRANSLATION_CONTROLLER_HPP_
