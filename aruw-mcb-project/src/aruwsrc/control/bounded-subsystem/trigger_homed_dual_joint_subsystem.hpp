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

#ifndef TRIGGER_HOMED_DUAL_JOINT_SUBSYSTEM_HPP_
#define TRIGGER_HOMED_DUAL_JOINT_SUBSYSTEM_HPP_

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/communication/sensors/limit_switch/limit_switch_interface.hpp"
#include "tap/motor/motor_interface.hpp"
#include "tap/util_macros.hpp"

#include "trigger_homed_joint_subsystem.hpp"

namespace aruwsrc::control
{
class TriggerHomedDualJointSubsystem : public TriggerHomedJointSubsystem
{
public:
    TriggerHomedDualJointSubsystem(
        tap::Drivers *drivers,
        tap::motor::MotorInterface &motorOne,
        tap::motor::MotorInterface &motorTwo,
        control::TriggerInterface &trigger,
        const tap::algorithms::SmoothPidConfig
            &alignPidConfig,
        const Config &config);

    void resetEncoderValue() override;

    float getPosition() const override;

    float getVelocity() const override;

    float getPositionDifference();

    float getVelocityDifference();

    void runPosPidController(float dt) override;

    void initialize() override;

    void refreshSafeDisconnect() override;

protected:
    /**
     * Stops the motor from moving. Only to be used during calibration.
     */
    void stopDuringHoming() override;

private:
    tap::motor::MotorInterface &motorOne,
        &motorTwo;  // motor one is stored as motor in parent class JointSubsystem
    tap::algorithms::SmoothPid alignPid;
};

}  // namespace aruwsrc::control

#endif  // TRIGGER_HOMED_DUAL_JOINT_SUBSYSTEM_HPP_
