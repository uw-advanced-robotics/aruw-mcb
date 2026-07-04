/*
 * Copyright (c) 2021-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "tap/architecture/timeout.hpp"  // add this include
#include "tap/control/command.hpp"

#include "aruwsrc/algorithms/odometry/otto_chassis_world_yaw_observer.hpp"
#include "aruwsrc/communication/sensors/encoder/fake_encoder.hpp"
#include "aruwsrc/control/chassis/holonomic_chassis_subsystem.hpp"
#include "aruwsrc/control/imu/imu_calibrate_command.hpp"

#include "engineer_turret_subsystem.hpp"

using namespace aruwsrc::control::turret::algorithms;
using namespace tap::algorithms::odometry;
namespace aruwsrc::engineer
{
class BinnedAlignmentCommand : public tap::control::Command
{
public:
    BinnedAlignmentCommand(
        aruwsrc::control::turret::TurretSubsystem &turret,
        tap::encoder::EncoderInterface &turretLampreyEncoder,
        tap::encoder::EncoderInterface &turretPulleyEncoder,
        tap::encoder::EncoderInterface &turretInternalEncoder,
        float velocityZeroThreshold,
        float binnedAlignmentOffset,
        float homeAlignmentOffset);

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override;

    const char *getName() const override { return "binned alignment"; }

protected:
    aruwsrc::control::turret::TurretSubsystem *turret;

    tap::encoder::EncoderInterface &turretLampreyEncoder;
    tap::encoder::EncoderInterface &turretPulleyEncoder;
    tap::encoder::EncoderInterface &turretInternalEncoder;

private:
    bool finishedAlignment;
    bool alignmentFailed = false;
    tap::arch::MilliTimeout calibrationLongTimeout;
    const float velocityZeroThreshold;
    const float binnedAlignmentOffset;
    const float homeAlignmentOffset;
    float curOffset;
    aruwsrc::communication::sensors::encoder::FakeEncoder fakeLampreyEncoder;

    inline bool turretNotMoving() const
    {
        return compareFloatClose(
                   0.0f,
                   turret->yawMotor.getChassisFrameVelocity(),
                   velocityZeroThreshold) &&
               compareFloatClose(
                   0.0f,
                   turret->pitchMotor.getChassisFrameVelocity(),
                   velocityZeroThreshold);
    };
};

}  // namespace aruwsrc::engineer

#endif  // BINNED_ALIGNMENT_COMMAND_HPP_