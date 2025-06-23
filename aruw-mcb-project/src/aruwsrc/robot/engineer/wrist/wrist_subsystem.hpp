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

#include "aruwsrc/robot/engineer/digital_out_subsystem.hpp"

namespace aruwsrc::engineer::wrist
{
struct WristConfig
{
    tap::algorithms::SmoothPidConfig pitchPidConfig;
    tap::algorithms::SmoothPidConfig yawPidConfig;

    // Minimum and maximum setpoints for pitch and yaw
    float minPitch = 0.0f;
    float maxPitch = 0.0f;
    float minYaw = 0.0f;
    float maxYaw = 0.0f;

    float ratio = 1.0f;     // differential pitch gear teeth / yaw gear teeth
    float epsilon = 1e-4f;  // angular tolerance used to determine if we reached the setpoint

    int32_t maxMotorDesiredOutput;
};

class WristSubsystem : public tap::control::Subsystem
{
public:
    WristSubsystem(
        tap::Drivers *drivers,
        tap::motor::MotorInterface &motorLeft,
        tap::motor::MotorInterface &motorRight,
        tap::encoder::EncoderInterface &encoderPitch,
        tap::encoder::EncoderInterface &encoderYaw,
        const aruwsrc::engineer::DigitalOutSubsystem &suction,
        const WristConfig config);

    float getPitch();

    float getYaw();

    float getSetpointPitch() { return setpointPitch; }

    float getSetpointYaw() { return setpointYaw; }

    void setSetpointPitch(float setpoint);

    void setSetpointYaw(float setpoint);

    virtual void initialize() override;

    bool atSetpointPitch(float epsilon = 1e-4);

    bool atSetpointYaw(float epsilon = 1e-4);

    bool atSetpoint();

    virtual void refresh() override;

    virtual void refreshSafeDisconnect() override;

private:
    tap::motor::MotorInterface &motorLeft, &motorRight;
    tap::encoder::EncoderInterface &encoderPitch, &encoderYaw;
    tap::algorithms::SmoothPid pidPitch, pidYaw;
    const aruwsrc::engineer::DigitalOutSubsystem &suction;
    const WristConfig config;

    float setpointPitch, setpointYaw;

    const tap::algorithms::transforms::Position COM_POS =
        tap::algorithms::transforms::Position(0.164, 0, 0.041);  // cant be static
    static constexpr float WRIST_MASS_KG = 0.4;
    const tap::algorithms::transforms::Position COM_POS_W_CUBE = COM_POS;
    static constexpr float WRIST_MASS_W_CUBE_KG = WRIST_MASS_KG;

    static constexpr float M3508_TORQUE_CONSTANT =
        (tap::motor::DjiMotor::MAX_OUTPUT_C620 / 20.0f) / 0.21f;  // desOut/A / (Nm/A) = desOut/Nm

    tap::algorithms::transforms::Transform computeWristToCOM(
        float yawJoint,
        float pitchJoint,
        tap::algorithms::transforms::Position COMPos) const;

    const tap::algorithms::transforms::Position getPosCOM() const;
    float getMass() const;
};
}  // namespace aruwsrc::engineer::wrist

#endif  // WRIST_SUBSYSTEM_HPP_