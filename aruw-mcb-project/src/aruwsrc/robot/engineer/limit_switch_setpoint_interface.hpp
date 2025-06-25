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

#ifndef LIMIT_SWITCH_SETPOINT_INTERFACE_HPP_
#define LIMIT_SWITCH_SETPOINT_INTERFACE_HPP_

#include "tap/algorithms/smooth_pid.hpp"

#include "aruwsrc/control/bounded-subsystem/one_sided_bounded_subsystem_interface.hpp"
#include "aruwsrc/control/bounded-subsystem/trigger/trigger_interface.hpp"
#include "aruwsrc/robot/engineer/linear_joint_interface.hpp"

namespace aruwsrc::engineer
{
enum class PIDState
{
    POSITION_PID,
    VELOCITY_PID,
    NONE
};

class LimitSwitchSetpointInterface : public aruwsrc::control::OneSidedBoundedSubsystemInterface,
                                     public LinearJointInterface
{
public:
    virtual void setDesiredOutput(int16_t output) = 0;

    virtual void resetEncoderValue() = 0;

    virtual float getEncoderValue() = 0;

    virtual float getEncoderVelocity() = 0;

    void setPIDState(PIDState state) { pidState = state; }

    PIDState getPIDState() { return pidState; }

    bool isTriggered() { return trigger.isTriggered(); }

    float getLowerBound() const override { return minSetpoint; }

    float getUpperBound() const override { return maxSetpoint; }

    float getPosition() override { return getEncoderValue() * radius; }

    float getSetpoint() { return setpoint; }

    float getVelocity() { return getEncoderVelocity() * radius; }

    void setHome(float home) override { this->home = home; };

    void setSetpoint(float setpoint) override
    {
        if (tap::algorithms::compareFloatClose(minSetpoint, maxSetpoint, epsilon) ||
            calibrationState != CalibrationState::CALIBRATION_COMPLETE)
            this->setpoint = setpoint;
        else
            this->setpoint = std::clamp(setpoint, minSetpoint, maxSetpoint);
    };

    bool homedAndBounded() const
    {
        return calibrationState == CalibrationState::CALIBRATION_COMPLETE;
    }

    // Acacia and Swara's setpoint interface logic
    void refresh() override
    {
        motorPos = getPosition();

        if (calibrationState == CalibrationState::CALIBRATING_LOWER_BOUND)
        {
            if (trigger.isTriggered())
            {
                calibrationState = CalibrationState::CALIBRATION_COMPLETE;
                resetEncoderValue();
                pidState = PIDState::POSITION_PID;
                setSetpoint(home);
            }
            else
            {
                pidState = PIDState::NONE;
                moveTowardLowerBound();
            }
        }

        if (pidState == PIDState::POSITION_PID)
        {
            float error = setpoint - motorPos;
            float errorDerivative = getVelocity();
            float newTime = tap::arch::clock::getTimeMilliseconds();
            float timeDifference = (newTime - lastTime) / 1000.0f;  // (s)
            lastTime = newTime;
            motorDesiredOutput = pid.runController(error, errorDerivative, timeDifference) + kS;
            setDesiredOutput(std::clamp(motorDesiredOutput, -maxOutput, maxOutput));
        }
        else if (pidState == PIDState::VELOCITY_PID)
        {
            // float error = velocitySetpoint -
            //               motor.getEncoder()->getVelocity() / 1000 / 60 / MM_PER_REVOLUTION /
            //               1000;
            // float timeDifference = (tap::arch::clock::getTimeMilliseconds() - lastTime) / 1000;
            // lastTime = tap::arch::clock::getTimeMilliseconds();
            // homingPID.runControllerDerivateError(error, timeDifference);
            // motor.setDesiredOutput(homingPID.getOutput());
            // TODO: fix math if we actually want to use
        }
        else
        {
            setDesiredOutput(std::clamp(motorDesiredOutput, -maxOutput, maxOutput));
        }
    }

    void refreshSafeDisconnect() override { setDesiredOutput(0); }

    void moveTowardLowerBound() override
    {
        // motorDesiredOutput = (homingReversed ? homingSpeed : -homingSpeed) + kS;
        pidState = PIDState::POSITION_PID;
        setSetpoint(getPosition() + (homingReversed ? homingSpeed : -homingSpeed));
    }

    void stopDuringHoming() override
    {
        motorDesiredOutput = 0;
        setDesiredOutput(0);
    }

protected:
    LimitSwitchSetpointInterface(
        tap::Drivers *drivers,
        aruwsrc::control::TriggerInterface &trigger,
        const tap::algorithms::SmoothPidConfig &pidConfig,
        float radius = 1.0f,
        float lowerBound = 0.0f,
        float upperBound = 0.0f,
        float home = 0.0f,
        float kS = 0,
        float epsilon = 0.5f,
        float homingSpeed = 0.25f,
        bool homingReversed = false,
        float maxOutput = 6000.0f)
        : OneSidedBoundedSubsystemInterface(drivers, trigger, 0),
          LinearJointInterface(lowerBound, upperBound, epsilon),
          pid(pidConfig),
          radius(radius),
          home(home),
          kS(kS),
          homingSpeed(homingSpeed),
          homingReversed(homingReversed),
          maxOutput(maxOutput)
    {
    }

    PIDState pidState = PIDState::POSITION_PID;
    tap::algorithms::SmoothPid pid;
    float radius;
    float home;
    float kS;
    float homingSpeed;
    bool homingReversed;
    float maxOutput;
    float lastTime = 0;
    float motorPos = 0;
    float motorDesiredOutput = 0;
};
}  // namespace aruwsrc::engineer

#endif