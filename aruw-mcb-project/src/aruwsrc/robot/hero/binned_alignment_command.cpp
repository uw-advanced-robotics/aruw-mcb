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