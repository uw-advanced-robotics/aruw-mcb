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
    BinnedAlignmentCommand(aruwsrc::hero::HeroTurretEncoderSubsystem& encoders, float localOffset);

    void initialize() override{};

    void execute() override;

    void end(bool) override{};

    bool isFinished() const override { return false; };

    const char* getName() const override { return "Binned Alignment Command"; }

private:
    HeroTurretEncoderSubsystem& encoders;
    float localOffset;
};

}  // namespace aruwsrc::hero

#endif  // BINNED_ALIGNMENT_COMMAND_HPP_