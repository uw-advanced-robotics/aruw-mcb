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

#ifndef TRIGGER_HOMED_JOINT_SUBSYSTEM_HPP_
#define TRIGGER_HOMED_JOINT_SUBSYSTEM_HPP_

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/motor/motor_interface.hpp"

#include "one_sided_bounded_subsystem_interface.hpp"
#include "trigger/trigger_interface.hpp"
#include "aruwsrc/control/joint/joint_subsystem.hpp"

namespace aruwsrc::control::joint::homing
{
class TriggerHomedJointSubsystem : public OneSidedBoundedSubsystemInterface,
                                   public JointSubsystem
{
public:
    struct Config
    {
        JointSubsystem::Config super;
        float home = 0.0f;
        float homingSpeed = 0.25f;
        bool homingReversed = false;
    };

    TriggerHomedJointSubsystem(
        tap::Drivers *drivers,
        tap::motor::MotorInterface &motor,
        trigger::TriggerInterface &trigger,
        Config config)
        : Subsystem(drivers),
          OneSidedBoundedSubsystemInterface(drivers, trigger, 0),
          JointSubsystem(drivers, motor, config.super),
          motor(motor),
          home(config.home),
          homingSpeed(config.homingSpeed),
          homingReversed(config.homingReversed)
    {
    }

    virtual inline void resetEncoderValue() { motor.getEncoder()->resetEncoderValue(); }

    inline float getLowerBound() const override { return lowerBound; }

    inline float getUpperBound() const override { return upperBound; }

    inline void setHome(float home) override { this->home = home; };

    void setSetpoint(float setpoint) override
    {
        if (tap::algorithms::compareFloatClose(lowerBound, upperBound, epsilon) ||
            calibrationState != CalibrationState::CALIBRATION_COMPLETE)
            this->setpoint.setTarget(setpoint);
        else
            this->setpoint.setTarget(std::clamp(setpoint, lowerBound, upperBound));
    };

    inline bool homedAndBounded() const
    {
        return calibrationState == CalibrationState::CALIBRATION_COMPLETE;
    }

    // AcaciaSwara's calibration logic
    void refresh() override
    {
        this->updateSetpoint();

        if (calibrationState == CalibrationState::CALIBRATING_LOWER_BOUND)
        {
            if (trigger.isTriggered())
            {
                calibrationState = CalibrationState::CALIBRATION_COMPLETE;
                resetEncoderValue();
                setSetpoint(home);
            }
            else
            {
                moveTowardLowerBound();
            }
        }

        runPosPidController(0.002f);
    }

    void moveTowardLowerBound() override
    {
        setSetpoint(getPosition() + (homingReversed ? homingSpeed : -homingSpeed));
    }

    void stopDuringHoming() override { motor.setDesiredOutput(0); }

protected:
    tap::motor::MotorInterface &motor;

    float home;
    float homingSpeed;
    bool homingReversed;
};
}  // namespace aruwsrc::control::joint::homing

#endif  // TRIGGER_HOMED_JOINT_SUBSYSTEM_HPP_