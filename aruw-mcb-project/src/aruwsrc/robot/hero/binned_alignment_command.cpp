#include "aruwsrc/robot/hero/binned_alignment_command.hpp"

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

    const float alignedPosition = algorithms::binned_encoder_alignment::calculatePosition<1, 2>(
        localEncoderPosition, globalEncoderPosition, localOffset);

    encoders.setEncoderPosition(alignedPosition);
}


}  // namespace aruwsrc::hero