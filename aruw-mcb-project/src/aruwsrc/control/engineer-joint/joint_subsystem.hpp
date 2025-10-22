/*
 * Copyright (c) 2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef JOINT_SUBSYSTEM_HPP_
#define JOINT_SUBSYSTEM_HPP_

#include "tap/algorithms/ramp.hpp"
#include "tap/algorithms/smooth_pid.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/motor/motor_interface.hpp"
#include "tap/util_macros.hpp"

namespace aruwsrc::control
{
/**
 * Subsystem code for joints that don't need to be homed.
 */
class JointSubsystem : public virtual tap::control::Subsystem
{
public:
    struct Config
    {
        float lowerBound = 0.0f, upperBound = 0.0f;
        float epsilon = 1e-4;
        float maxSetpointIncrement = FLT_MAX;
        float initSetpoint = 0;

        // Conversion factor from encoder measurement to joint position, assuming linear
        // relationship (e.g. pulley radius for a prismatic joint, gear ratio for a rotary joint,
        // etc.)
        float encoderRatio = 1.0f;

        tap::algorithms::SmoothPidConfig posPidConfig;
        float maxOutput;
        float staticFeedforward = 0.0f;
    };

    JointSubsystem(tap::Drivers *drivers, tap::motor::MotorInterface &motor, Config config);

    void initialize() override;

    virtual void runPosPidController(float dt);

    void refresh() override;

    void refreshSafeDisconnect() override;

    virtual void setSetpoint(float setpoint);

    inline void updateSetpoint() { setpoint.update(maxSetpointIncrement); }

    float getSetpoint() const { return setpoint.getValue(); }

    virtual float getPosition() const;

    virtual float getVelocity() const;

    bool atSetpoint();

    float getLowerBound() const { return lowerBound; }

    float getUpperBound() const { return upperBound; }

    void setLowerBound(float lowerBound);

    void setUpperBound(float upperBound);

protected:
    tap::algorithms::Ramp setpoint;
    float lowerBound, upperBound;
    tap::motor::MotorInterface &motor;
    const float epsilon, maxSetpointIncrement;
    tap::algorithms::SmoothPid posPid;
    float encoderRatio;
    float staticFeedforward;
    float maxOutput;
};
}  // namespace aruwsrc::control

#endif  // JOINT_SUBSYSTEM_HPP_