#ifndef TURRET_ENCODER_TRANSFORM_OBSERVER_HPP_
#define TURRET_ENCODER_TRANSFORM_OBSERVER_HPP_

#include "aruwsrc/algorithms/state/transform_observer_interface.hpp"

#include "turret_subsystem.hpp"

namespace aruwsrc::control::turret
{
template <aruwsrc::algorithms::state::Frame MOUNTING_FRAME>
class TurretEncoderTransformObserver
    : public aruwsrc::algorithms::state::
          TransformObserverInterface<MOUNTING_FRAME, aruwsrc::algorithms::state::Frame::TURRET>
{
public:
    TurretEncoderTransformObserver(const TurretSubsystem& turret) : turret(turret) {}

    tap::algorithms::transforms::DynamicOrientation getOrientation() const override
    {
        return tap::algorithms::transforms::DynamicOrientation(
            0.0f,
            turret.pitchMotor.getChassisFrameMeasuredAngle().getWrappedValue(),
            turret.yawMotor.getChassisFrameMeasuredAngle().getWrappedValue(),
            0.0f,
            turret.pitchMotor.getChassisFrameVelocity(),
            turret.yawMotor.getChassisFrameVelocity());
    }

    tap::algorithms::transforms::DynamicPosition getTranslation() const override
    {
        return tap::algorithms::transforms::DynamicPosition(
            turret.getTurretOffset().coordinates(),
            Vector(0, 0, 0).coordinates(),
            Vector(0, 0, 0).coordinates());
    }

    inline bool isOnline() const override { return turret.isOnline(); }

private:
    const TurretSubsystem& turret;
};  // TurretEncoderTransformObserver

}  // namespace aruwsrc::control::turret

#endif  // TURRET_ENCODER_TRANSFORM_OBSERVER_HPP_