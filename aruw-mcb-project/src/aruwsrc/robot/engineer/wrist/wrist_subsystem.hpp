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
    // theta 3 is on very end and ONLY "roll"
    tap::algorithms::SmoothPidConfig theta1PidConfig;
    tap::algorithms::SmoothPidConfig theta2PidConfig;
    tap::algorithms::SmoothPidConfig theta3PidConfig;

    float theta2Min;
    float theta2Max;

    float ratio = 1.0f;     // differential pitch gear teeth / yaw gear teeth
    float epsilon = 1e-4f;  // angular tolerance used to determine if we reached the setpoint

    int32_t maxMotorDesiredOutput;
};

class WristSubsystem : public tap::control::Subsystem
{
public:
    WristSubsystem(
        tap::Drivers* drivers,
        tap::motor::MotorInterface& motorDifferential1,
        tap::motor::MotorInterface& motorDifferential2,
        tap::motor::MotorInterface& motorTheta3,
        tap::encoder::EncoderInterface& encoderTheta2,
        const WristConfig config);

    float getTheta1() const;
    float getTheta2() const;
    float getTheta3() const;

    void setSetpointTheta1(float setpoint);
    void setSetpointTheta2(float setpoint);
    void setSetpointTheta3(float setpoint);

    void homeTheta3(float currPos);

    /**
     * Sets the desired rotation for the entire wrist
     * */
    void setSetpointOrientation(tap::algorithms::transforms::Orientation setpoint);

    inline float getSetpointTheta1() const { return setpointTheta1.getWrappedValue(); }
    inline float getSetpointTheta2() const { return setpointTheta2; }
    inline float getSetpointTheta3() const { return setpointTheta3.getWrappedValue(); }

    tap::algorithms::transforms::Orientation getSetpointOrientation() const;

    virtual void initialize() override;

    bool atSetpointTheta1(float epsilon = 1e-4) const;

    bool atSetpointTheta2(float epsilon = 1e-4) const;

    bool atSetpointTheta3(float epsilon = 1e-4) const;

    bool atSetpoint() const;

    virtual void refresh() override;

    virtual void refreshSafeDisconnect() override;

    bool isOnline() const;

    tap::algorithms::transforms::Orientation getOrientation() const;

    static tap::algorithms::transforms::Orientation getHypotheticalOrientation(
        float theta1,
        float theta2,
        float theta3);

private:
    const WristConfig config;
    tap::motor::MotorInterface &motorDifferential1, &motorDifferential2, &motorTheta3;
    tap::encoder::EncoderInterface& encoderTheta2;
    tap::algorithms::WrappedFloat setpointTheta1;
    float setpointTheta2;
    tap::algorithms::WrappedFloat setpointTheta3;
    tap::algorithms::SmoothPid pidTheta1, pidTheta2, pidTheta3;
};
}  // namespace aruwsrc::engineer::wrist

#endif  // WRIST_SUBSYSTEM_HPP_