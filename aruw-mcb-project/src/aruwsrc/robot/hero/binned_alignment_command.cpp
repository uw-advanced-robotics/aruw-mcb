/*
 * Copyright (c) 2026 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "aruwsrc/robot/hero/binned_alignment_command.hpp"

#include "aruwsrc/control/turret/constants/turret_constants.hpp"

namespace aruwsrc::hero
{
BinnedAlignmentCommand::BinnedAlignmentCommand(HeroTurretEncoders& encoders, float localOffset)
    : encoders(encoders),
      localOffset(localOffset)
{
}

void BinnedAlignmentCommand::initialize()
{
    const float localEncoderPosition = encoders.getYawMotorPosition();
    const float globalEncoderPosition = encoders.getYawLampreyPosition();

    // since the lamprey isnt on the hero yet, we could jsut test with something like
    // globalEncoderPosition = 0

    const float alignedPosition = algorithms::binned_encoder_alignment::calculatePosition<
        aruwsrc::control::turret::ENCODER_RATIO_NUM,
        aruwsrc::control::turret::ENCODER_RATIO_DEN>(
        localEncoderPosition,
        globalEncoderPosition,
        localOffset);

    encoders.setEncoderPosition(alignedPosition);
}

}  // namespace aruwsrc::hero