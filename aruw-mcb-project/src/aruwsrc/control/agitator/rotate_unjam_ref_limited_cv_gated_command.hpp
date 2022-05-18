/*
 * Copyright (c) 2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef ROTATE_UNJAM_REF_LIMITED_CV_GATED_COMMAND_HPP_
#define ROTATE_UNJAM_REF_LIMITED_CV_GATED_COMMAND_HPP_

#include "aruwsrc/control/agitator/rotate_unjam_ref_limited_command.hpp"
#include "aruwsrc/control/auto-aim/auto_aim_launch_timer.hpp"
#include "aruwsrc/control/turret/cv/turret_cv_command.hpp"

namespace aruwsrc
{
class Drivers;
}

namespace aruwsrc::agitator
{
/**
 * An extension to the standard RotateUnjamRefLimitedCommand which gates firing on computer vision
 * aim.
 */
class RotateUnjamRefLimitedCvGatedCommand
    : public aruwsrc::agitator::RotateUnjamRefLimitedCommand
{
public:
    RotateUnjamRefLimitedCvGatedCommand(
        aruwsrc::Drivers &drivers,
        tap::control::setpoint::IntegrableSetpointSubsystem &subsystem,
        tap::control::setpoint::MoveIntegralCommand &moveIntegralCommand,
        tap::control::setpoint::UnjamIntegralCommand &unjamCommand,
        aruwsrc::control::turret::cv::TurretCVCommand &turretCVCommand,
        uint16_t heatLimitBuffer,
        aruwsrc::control::auto_aim::AutoAimLaunchTimer& autoAimLaunchTimer,
        uint8_t turretId
        );

    bool isReady() override;

private:
    aruwsrc::control::turret::cv::TurretCVCommand& turretCVCommand;
    aruwsrc::control::auto_aim::AutoAimLaunchTimer& autoAimLaunchTimer;
    uint8_t turretId;
};

}  // namespace aruwsrc::agitator

#endif  // ROTATE_UNJAM_REF_LIMITED_CV_GATED_COMMAND_HPP_
