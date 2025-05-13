/*
 * Copyright (c) 2025-2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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