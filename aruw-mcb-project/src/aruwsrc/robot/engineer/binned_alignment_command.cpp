/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of Taproot.
 *
 * Taproot is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Taproot is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Taproot.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "binned_alignment_command.hpp"

#include "tap/drivers.hpp"

#include "aruwsrc/algorithms/binned_encoder_alignment/binned_encoder_alignment.hpp"
#include "aruwsrc/control/turret/constants/turret_constants.hpp"
#include "aruwsrc/control/turret/yaw_turret_subsystem.hpp"

using namespace tap::algorithms;

namespace aruwsrc::engineer
{
static constexpr uint32_t MAX_ALIGNMENT_WAITTIME_MS = 5000;  // TODO: tune

BinnedAlignmentCommand::BinnedAlignmentCommand(
    control::turret::TurretSubsystem &turret,
    tap::encoder::EncoderInterface &turretLampreyEncoder,
    tap::encoder::EncoderInterface &turretPulleyEncoder,
    tap::encoder::EncoderInterface &turretInternalEncoder,
    float velocityZeroThreshold,
    const float binnedAlignmentOffset,
    const float homeAlignmentOffset)
    : turret(&turret),
      turretLampreyEncoder(turretLampreyEncoder),
      turretPulleyEncoder(turretPulleyEncoder),
      turretInternalEncoder(turretInternalEncoder),
      velocityZeroThreshold(velocityZeroThreshold),
      binnedAlignmentOffset(binnedAlignmentOffset),
      homeAlignmentOffset(homeAlignmentOffset),
      fakeLampreyEncoder(0, 0)
{
    addSubsystemRequirement(&turret);
}

void BinnedAlignmentCommand::initialize()
{
    calibrationLongTimeout.restart(MAX_ALIGNMENT_WAITTIME_MS);
    finishedAlignment = false;
    alignmentFailed = false;
}

void BinnedAlignmentCommand::execute()
{
    if (calibrationLongTimeout.isExpired())
    {
        alignmentFailed = true;
        return;
    }

    if (!turretPulleyEncoder.isOnline() || !turretLampreyEncoder.isOnline() || !turretNotMoving())
    {
        return;
    }

    curOffset = aruwsrc::algorithms::binned_encoder_alignment::calculateOffset<4, 15>(
        turretPulleyEncoder.getPosition().getWrappedValue(),
        turretLampreyEncoder.getPosition().getWrappedValue());

    fakeLampreyEncoder.setFakePosition(
        aruwsrc::algorithms::binned_encoder_alignment::calculatePosition<4, 15>(
            turretPulleyEncoder.getPosition().getWrappedValue(),
            turretLampreyEncoder.getPosition().getWrappedValue(),
            binnedAlignmentOffset) -
        homeAlignmentOffset);

    turretInternalEncoder.alignWith(&fakeLampreyEncoder);
    finishedAlignment = true;
}

bool BinnedAlignmentCommand::isFinished() const { return finishedAlignment || alignmentFailed; }

void BinnedAlignmentCommand::end(bool) {}

}  // namespace aruwsrc::engineer