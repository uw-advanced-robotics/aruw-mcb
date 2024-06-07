/*
 * Copyright (c) 2024-2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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
#ifndef SENTRY_MINOR_WORLD_ORIENTATION_PROVIDER_HPP_
#define SENTRY_MINOR_WORLD_ORIENTATION_PROVIDER_HPP_

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/algorithms/transforms/transform.hpp"
#include "tap/control/subsystem.hpp"

#include "aruwsrc/communication/can/turret_mcb_can_comm.hpp"
#include "aruwsrc/control/turret/turret_motor.hpp"

using namespace tap::algorithms;

namespace aruwsrc::control::turret
{

class SentryMinorWorldOrientationProvider
{
public:
    SentryMinorWorldOrientationProvider(
        const TurretMotor& turretYawMotor,
        const aruwsrc::can::TurretMCBCanComm& turretMCB,
        tap::communication::sensors::imu::ImuInterface& majorImu,
        const SmoothPidConfig& yawCorrectionPidConfig);

    void initialize();

    void update();

    /**
     * Resets the system to assume everything is at zero
     */
    void zero();

    inline WrappedFloat getYaw() const { return Angle(turretMCB.getYaw()) + yawCorrection; }

    inline float getYawVelocity() const { return turretMCB.getYawVelocity(); }

    inline WrappedFloat getPitch() const
    {
        return Angle(turretMCB.getPitch());
    }  // todo: this might need correction too

    inline float getPitchVelocity() const { return turretMCB.getPitchVelocity(); }

    inline WrappedFloat getRoll() const { return Angle(turretMCB.getRollVelocity()); }

    inline float getRollVelocity() const { return turretMCB.getRollVelocity(); }

    inline transforms::Orientation getOrientation() const
    {
        return transforms::Orientation(
            getRoll().getWrappedValue(),
            getPitch().getWrappedValue(),
            getYaw().getWrappedValue());
    }

private:
    WrappedFloat getBaselineYaw() const;

    const TurretMotor& turretYawMotor;
    const aruwsrc::can::TurretMCBCanComm& turretMCB;
    tap::communication::sensors::imu::ImuInterface& majorImu;

    tap::algorithms::SmoothPid yawCorrectionPid;

    float yawCorrection{0};

    uint32_t prevtimeMillis{0};

};  // class SentryMinorWorldOrientationProvider

}  // namespace aruwsrc::control::turret
#endif  // SENTRY_MINOR_WORLD_ORIENTATION_PROVIDER_HPP_
