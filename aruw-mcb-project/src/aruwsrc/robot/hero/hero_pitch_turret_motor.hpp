/*
 * Copyright (c) 2020-2026 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef HERO_PITCH_TURRET_MOTOR_HPP_
#define HERO_PITCH_TURRET_MOTOR_HPP_

#include "aruwsrc/control/turret/turret_motor.hpp"

namespace aruwsrc::hero
{
class HeroPitchLinkage : public aruwsrc::control::turret::TurretMotor
{
public:
    struct FourBarLinkageConfig
    {
        float l1, l2, l3, l4;
    };

    HeroPitchLinkage(
        tap::motor::MotorInterface *motor,
        const aruwsrc::control::turret::TurretMotorConfig &motorConfig,
        const FourBarLinkageConfig &linkageConfig)
        : aruwsrc::control::turret::TurretMotor(motor, motorConfig),
          linkageConfig(linkageConfig){};

    void updateMotorAngle() override
    {
        if (isOnline())
        {
            const float l1 = linkageConfig.l1;
            const float l2 = linkageConfig.l2;
            const float l3 = linkageConfig.l3;
            const float l4 = linkageConfig.l4;

            float theta1 = -motor->getEncoder()->getPosition().getWrappedValue() + M_TWOPI + M_PI;
            while (theta1 > M_TWOPI)
            {
                theta1 -= M_TWOPI;
            }

            const float theta2 = theta1 - M_PI_2;

            const float v1 = sqrt(l4 * l4 + l1 * l1 - 2 * (l4 * l2) * cos(theta2));

            const float theta3 = acos((l3 * l3 - l2 * l2 - v1 * v1) / (-2.0f * l2 * v1));

            const float theta5 = acos((l4 * l4 - v1 * v1 - l1 * l1) / (-2.0f * v1 * l1));

            const float pos = theta5 + theta3 - M_PI_2 + config.startAngle;

            chassisFrameMeasuredAngle.setWrappedValue(pos);
        }
        else
        {
            chassisFrameMeasuredAngle.setUnwrappedValue(config.startAngle);
        }
    }

    const FourBarLinkageConfig &linkageConfig;
};
}  // namespace aruwsrc::hero

#endif  // HERO_PITCH_TURRET_MOTOR_