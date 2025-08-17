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

#ifndef GANTRY_LIFT_SUBSYSTEM_HPP_
#define GANTRY_LIFT_SUBSYSTEM_HPP_

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/communication/sensors/limit_switch/limit_switch_interface.hpp"
#include "tap/motor/motor_interface.hpp"
#include "tap/util_macros.hpp"

#include "aruwsrc/robot/engineer/limit_switch_setpoint_interface.hpp"

namespace aruwsrc::engineer::gantry
{
class GantryLiftSubsystem : public LimitSwitchSetpointInterface
{
public:
    GantryLiftSubsystem(
        tap::Drivers *drivers,
        tap::motor::MotorInterface &motorLeft,
        tap::motor::MotorInterface &motorRight,
        const tap::algorithms::SmoothPidConfig &configPos,
        const tap::algorithms::SmoothPidConfig &configAlign,
        control::TriggerInterface &trigger,
        const LimitSwitchSetpointInterface::LimitSwitchConfig &limitConfig);

    void setDesiredOutput(int16_t power) override;

    void resetEncoderValue() override;

    float getEncoderValue() override;

    float getEncoderVelocity() override;

    float getPositionDifference();

    float getVelocityDifference();

    void initialize() override;

    void refreshSafeDisconnect() override;

protected:
    /**
     * Stops the motor from moving. Only to be used during calibration.
     */
    virtual void stopDuringHoming() override;

private:
    tap::algorithms::SmoothPid pidAlign;
    tap::motor::MotorInterface &motorLeft, &motorRight;
};

}  // namespace aruwsrc::engineer::gantry

#endif  // GANTRY_LIFT_SUBSYSTEM_HPP_
