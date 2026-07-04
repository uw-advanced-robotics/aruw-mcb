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