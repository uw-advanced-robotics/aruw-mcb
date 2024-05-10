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
#ifndef SENTRY_MINOR_WORLD_ORIENTATION_PROVIDER_SUBSYSTEM_HPP_
#define SENTRY_MINOR_WORLD_ORIENTATION_PROVIDER_SUBSYSTEM_HPP_

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/control/subsystem.hpp"

#include "aruwsrc/communication/can/turret_mcb_can_comm.hpp"
#include "aruwsrc/control/turret/turret_motor.hpp"

namespace aruwsrc::control::turret
{

class SentryMinorWorldOrientationProviderSubsystem : public tap::control::Subsystem
{
public:
    SentryMinorWorldOrientationProviderSubsystem(
        TurretMotor& turretMotor,
        const aruwsrc::can::TurretMCBCanComm& turretMCB,
        const tap::algorithms::SmoothPidConfig& driftPidConfig,
        tap::Drivers* drivers);

    void initialize() override;

    void refresh() override;

    float getYaw() const;

    inline float getYawVel() const { return turretMCB.getYawVelocity(); };

    inline float getPitch() const { return turretMCB.getPitch(); }

    inline float getPitchVel() const { return turretMCB.getPitchVelocity(); }

    inline float getRoll() const { return 0.0; }

    inline float getRollVel() const { return 0.0; }

    inline void refreshSafeDisconnect() override {};

    const char* getName() const override
    {
        return "Sentry Minor World Orientation Provider Subsystem";
    }

private:
    TurretMotor turretMotor;
    const aruwsrc::can::TurretMCBCanComm& turretMCB;

    tap::algorithms::SmoothPid driftPid;

    // "encoder yaw" refers to the yaw computed using the turret major imu and turret minor yaw
    // encoder
    float lastIMUYaw{0}, lastEncoderYaw{0};

};  // class SentryMinorWorldOrientationProviderSubsystem

}  // namespace aruwsrc::control::turret
#endif  // SENTRY_MINOR_WORLD_ORIENTATION_PROVIDER_SUBSYSTEM_HPP_
