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

#ifndef ARM_LIFT_SUBSYSTEM_HPP_
#define ARM_LIFT_SUBSYSTEM_HPP_

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/algorithms/smooth_pid.hpp"
#include "tap/communication/sensors/limit_switch/limit_switch_interface.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/motor/motor_interface.hpp"
#include "tap/util_macros.hpp"

#include "limit_switch_setpoint_interface.hpp"

namespace aruwsrc
{
namespace engineer
{
class ArmLiftSubsystem : public LimitSwitchSetpointInterface
{
public:
    ArmLiftSubsystem(
        tap::Drivers *drivers,
        tap::motor::MotorInterface &motorLeft,
        tap::motor::MotorInterface &motorRight,
        const tap::algorithms::SmoothPidConfig &configPos,
        const tap::algorithms::SmoothPidConfig &configAlign,
        control::TriggerInterface &trigger,
        float radius,
        float lowerBound = 0.0f,
        float upperBound = 0.0f,
        float kS = 0.0f,
        float epsilon = 1e-4f);

    virtual float getPosition() override;

    float getPositionDifference();

    float getAverageVelocity();

    float getVelocityDifference();

    virtual void initialize() override;

    virtual void refresh() override;

    virtual void refreshSafeDisconnect() override;

    virtual void moveTowardLowerBound() override;

    virtual void setDesiredOutput(int16_t output) override
    {
        motorLeft.setDesiredOutput(output);
        motorRight.setDesiredOutput(output);
    }

protected:
    /**
     * Stops the motor from moving. Only to be used during calibration.
     */
    virtual void stopDuringHoming() override;
    /**
     * Sets the given motor encoder position to be the "home" of the subsystem's motor.
     */
    virtual void setHome(uint64_t encoderPosition) override
    {
        home = (encoderPosition * M_TWOPI / 4096.0f) * radius;
    }

private:
    tap::algorithms::SmoothPid pidPos, pidAlign;
    tap::motor::MotorInterface &motorLeft, &motorRight;
    float radius;
    float home;
    // Constant added to output to overcome static friction
    float kS;
};

}  // namespace engineer
}  // namespace aruwsrc

#endif  // ARM_LIFT_SUBSYSTEM_HPP_
