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
#include "aruwsrc/robot/engineer/linear_pid_interface.hpp"

namespace aruwsrc::engineer
{
enum class PIDState
{
    POSITION_PID,
    VELOCITY_PID,
    NONE
};

class LimitSwitchSetpointInterface : public aruwsrc::control::OneSidedBoundedSubsystemInterface,
                                     public LinearPIDInterface
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

    float getPosition() override { return getEncoderValue() * limitSwitchConfig.radius; }

    float getVelocity() { return getEncoderVelocity() * limitSwitchConfig.radius; }

    void setHome(float home) override { this->limitSwitchConfig.home = home; };

    void setSetpoint(float setpoint) override
    {
        if (tap::algorithms::compareFloatClose(minSetpoint, maxSetpoint, epsilon) ||
            calibrationState != CalibrationState::CALIBRATION_COMPLETE)
            this->setpoint.setTarget(setpoint);
        else
            this->setpoint.setTarget(std::clamp(setpoint, minSetpoint, maxSetpoint));
    };

    bool homedAndBounded() const
    {
        return calibrationState == CalibrationState::CALIBRATION_COMPLETE;
    }

    // AcaciaSwara's calibration logic
    void refresh() override
    {
        this->updateSetpoint();
        motorPos = getPosition();

        if (calibrationState == CalibrationState::CALIBRATING_LOWER_BOUND)
        {
            if (trigger.isTriggered())
            {
                calibrationState = CalibrationState::CALIBRATION_COMPLETE;
                resetEncoderValue();
                pidState = PIDState::POSITION_PID;
                setSetpoint(limitSwitchConfig.home);
            }
            else
            {
                moveTowardLowerBound();
            }
        }

        if (pidState == PIDState::POSITION_PID)
        {
            float error = setpoint.getValue() - motorPos;
            float errorDerivative = getVelocity();
            float newTime = tap::arch::clock::getTimeMilliseconds();
            float timeDifference = (newTime - lastTime) / 1000.0f;  // (s)
            lastTime = newTime;
            motorDesiredOutput =
                pid.runController(error, errorDerivative, timeDifference) + limitSwitchConfig.kS;
            setDesiredOutput(std::clamp(
                motorDesiredOutput,
                -limitSwitchConfig.maxOutput,
                limitSwitchConfig.maxOutput));
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
            setDesiredOutput(std::clamp(
                motorDesiredOutput,
                -limitSwitchConfig.maxOutput,
                limitSwitchConfig.maxOutput));
        }
    }

    void refreshSafeDisconnect() override { setDesiredOutput(0); }

    void moveTowardLowerBound() override
    {
        // motorDesiredOutput = (homingReversed ? homingSpeed : -homingSpeed) + kS;
        pidState = PIDState::POSITION_PID;
        setSetpoint(
            getPosition() + (limitSwitchConfig.homingReversed ? limitSwitchConfig.homingSpeed
                                                              : -limitSwitchConfig.homingSpeed));
    }

    void stopDuringHoming() override
    {
        motorDesiredOutput = 0;
        setDesiredOutput(0);
    }

    struct LimitSwitchConfig
    {
        float radius = 1.0f;
        float lowerBound = 0.0f;
        float upperBound = 0.0f;
        float home = 0.0f;
        float kS = 0;
        float epsilon = 0.5f;
        float homingSpeed = 0.25f;
        bool homingReversed = false;
        float maxOutput = 6000.0f;
        float maxSetpointIncrement = FLT_MAX;
    };

protected:
    LimitSwitchSetpointInterface(
        tap::Drivers *drivers,
        aruwsrc::control::TriggerInterface &trigger,
        const tap::algorithms::SmoothPidConfig &pidConfig,
        LimitSwitchConfig limitSwitchConfig)
        : OneSidedBoundedSubsystemInterface(drivers, trigger, 0),
          LinearPIDInterface(
              limitSwitchConfig.lowerBound,
              limitSwitchConfig.upperBound,
              limitSwitchConfig.epsilon,
              0,
              limitSwitchConfig.maxSetpointIncrement),
          pid(pidConfig),
          limitSwitchConfig(limitSwitchConfig)
    {
    }

    PIDState pidState = PIDState::POSITION_PID;
    tap::algorithms::SmoothPid pid;
    LimitSwitchConfig limitSwitchConfig;
    float lastTime = 0;
    float motorPos = 0;
    float motorDesiredOutput = 0;
};
}  // namespace aruwsrc::engineer

#endif