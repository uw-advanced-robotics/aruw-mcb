
/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef KICKER_AGITATOR_SUBSYSTEM_HPP_
#define KICKER_AGITATOR_SUBSYSTEM_HPP_

#include "agitator_subsystem.hpp"

class KickerAgitatorSubsystem : public aruwsrc::agitator::AgitatorSubsystem
{
public:
    /** Speed that the kicker will rotate, in radians / second */
    static constexpr float KICKER_ROTATE_SPEED_RAD_PER_S = 1.0f;
    static constexpr float ROTATE_SETPOINT_IF_NO_REF_SERIAL = 2.0f * M_PI;
    static constexpr int HEAT_LIMIT_DIFF_PROJECTILE_LAUNCHED = 90;

    KickerAgitatorSubsystem(
        aruwsrc::Drivers *drivers,
        float kp,
        float ki,
        float kd,
        float maxIAccum,
        float maxOutput,
        float agitatorGearRatio,
        tap::motor::MotorId agitatorMotorId,
        tap::can::CanBus agitatorCanBusId,
        bool isAgitatorInverted);

    void refresh() override;

    const char *getName() override { return "Hero firing system"; }

    void launchOneProjectile();

    bool isProjectileQueued() const { return projectileQueued; }

private:
    bool projectileQueued = false;
    bool rotateKicker = false;
    float startRotateSetpoint = 0.0f;
    uint32_t prevTime = 0;
    int32_t prevHeat42 = 0;

    void updateProjectileQueued();
    void updateKickerSetpoint();
    bool launchedProjectile();
};

#endif  // KICKER_AGITATOR_SUBSYSTEM_HPP_
