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

#ifndef FRAME_RELATIVE_CHASSIS_TRANSLATION_CONTROLLER_HPP_
#define FRAME_RELATIVE_CHASSIS_TRANSLATION_CONTROLLER_HPP_

#include "tap/algorithms/transforms/transform.hpp"

#include "chassis_translation_controller_interface.hpp"

namespace aruwsrc::control::chassis::controller
{
/**
 * A common operation is to have the chassis translate in either turret or world frame (i.e. not
 * chassis frame). This class handles that transform.
 */
class FrameRelativeChassisTranslationController : public ChassisTranslationControllerInterface
{
public:
    FrameRelativeChassisTranslationController(
        const tap::algorithms::transforms::Transform& frameToChassis)
        : frameToChassis(frameToChassis)
    {
    }

    tap::algorithms::transforms::Vector runTranslationController(
        const float maxSpeed) final override
    {
        return frameToChassis.apply(runFrameRelativeTranslationController(maxSpeed));
    }

    virtual tap::algorithms::transforms::Vector runFrameRelativeTranslationController(
        const float maxSpeed) = 0;

private:
    const tap::algorithms::transforms::Transform& frameToChassis;
};  // class ChassisYawControllerInterface

}  // namespace aruwsrc::control::chassis::controller

#endif  // FRAME_RELATIVE_CHASSIS_TRANSLATION_CONTROLLER_HPP_
