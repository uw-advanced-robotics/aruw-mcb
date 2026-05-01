#ifndef BINNED_ALIGNMENT_COMMAND_HPP_
#define BINNED_ALIGNMENT_COMMAND_HPP_

#include "tap/control/command.hpp"

#include "aruwsrc/algorithms/binned_encoder_alignment/binned_encoder_alignment.hpp"
#include "aruwsrc/robot/hero/hero_turret_encoders.hpp"
namespace aruwsrc::hero
{
class BinnedAlignmentCommand : public tap::control::Command
{
public:
    BinnedAlignmentCommand(aruwsrc::hero::HeroTurretEncoders& encoders, float localOffset);

    void initialize() override;

    void execute() override {};

    void end(bool) override {};

    bool isFinished() const override {};

    const char* getName() const override { return "Binned Alignment Command"; }



private:
    HeroTurretEncoders& encoders;
    float localOffset;
};

}  // namespace aruwsrc::hero

#endif  // BINNED_ALIGNMENT_COMMAND_HPP_