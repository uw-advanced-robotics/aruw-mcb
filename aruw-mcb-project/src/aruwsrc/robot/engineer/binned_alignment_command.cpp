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