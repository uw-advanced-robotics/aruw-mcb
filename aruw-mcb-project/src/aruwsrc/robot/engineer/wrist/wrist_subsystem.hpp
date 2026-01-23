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

#ifndef WRIST_SUBSYSTEM_HPP_
#define WRIST_SUBSYSTEM_HPP_

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/algorithms/transforms/transform.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/motor/motor_interface.hpp"
#include "tap/util_macros.hpp"

namespace aruwsrc::engineer::wrist
{
struct WristConfig
{
    // Joints ordered based on distance from base of wrist
    // theta 1 is "azimuth/roll", theta 2 is "pitch"
    tap::algorithms::SmoothPidConfig theta1PidConfig;
    tap::algorithms::SmoothPidConfig theta2PidConfig;

    float theta1Min = 0.0f;
    float theta1Max = 0.0f;
    float theta2Min = 0.0f;
    float theta2Max = 0.0f;

    float ratio = 1.0f;     // differential pitch gear teeth / yaw gear teeth
    float epsilon = 1e-4f;  // angular tolerance used to determine if we reached the setpoint

    int32_t maxMotorDesiredOutput;
};

class WristSubsystem : public tap::control::Subsystem
{
public:
    WristSubsystem(
        tap::Drivers* drivers,
        tap::motor::MotorInterface& motorLeft,
        tap::motor::MotorInterface& motorRight,
        tap::encoder::EncoderInterface& encoderTheta1,
        tap::encoder::EncoderInterface& encoderTheta2,
        const WristConfig config);

    float getTheta1();
    float getTheta2();
    float setSetpointTheta1(float setpoint);
    float setSetpointTheta2(float setpoint);
    float getSetpointTheta1() { return setpointTheta1; }
    float getSetpointTheta2() { return setpointTheta2; }

    float calculateLeftMotorOutputForTheta1Theta2(float theta1Setpoint, float theta2Setpoint);
    float calculateRightMotorOutputForTheta1Theta2(float theta1Setpoint, float theta2Setpoint);

    float getPitch();

    float getYaw();

    virtual void initialize() override;

    bool atSetpointTheta1(float epsilon = 1e-4);

    bool atSetpointTheta2(float epsilon = 1e-4);

    bool atSetpoint();

    virtual void refresh() override;

    virtual void refreshSafeDisconnect() override;

private:
    tap::motor::MotorInterface &motorLeft, &motorRight;
    tap::encoder::EncoderInterface &encoderTheta1, &encoderTheta2;
    tap::algorithms::SmoothPid pidTheta1, pidTheta2;
    const WristConfig config;

    float setpointTheta1, setpointTheta2;

    const tap::algorithms::transforms::Position COM_POS =
        tap::algorithms::transforms::Position(0.164, 0, 0.041);  // cant be static
    static constexpr float WRIST_MASS_KG = 0.4;
    static constexpr float M3508_TORQUE_CONSTANT =
        (tap::motor::DjiMotor::MAX_OUTPUT_C620 / 20.0f) / 0.21f;  // desOut/A / (Nm/A) = desOut/Nm

    tap::algorithms::transforms::Transform computeWristToCOM(
        float yawJoint,
        float pitchJoint,
        tap::algorithms::transforms::Position COMPos) const;
};
}  // namespace aruwsrc::engineer::wrist

#endif  // WRIST_SUBSYSTEM_HPP_