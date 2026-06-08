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

#ifndef SERVO_SUBSYSTEM_HPP_
#define SERVO_SUBSYSTEM_HPP_

#include "tap/control/subsystem.hpp"

#include "aruwsrc/communication/mcb-lite/mcb_lite.hpp"
#include "aruwsrc/communication/mcb-lite/motor/virtual_servo.hpp"

namespace aruwsrc::engineer::servo
{
class VTMServoSubsystem : public tap::control::Subsystem
{
public:
    VTMServoSubsystem(tap::Drivers* drivers, aruwsrc::communication::mcb_lite::MCBLite& mcbLite, const aruwsrc::engineer::algorithms::EngineerTransforms& transforms)
        : tap::control::Subsystem(drivers),
          yawServo(drivers, tap::gpio::Pwm::Pin::X, 1.0f, 0.0f, 0.01f, mcbLite.pwm),
          pitchServo(drivers, tap::gpio::Pwm::Pin::Buzzer, 1.0f, 0.0f, 0.01f, mcbLite.pwm),
          mcbLite(mcbLite),
          transforms(transforms)
    {
    }

    void refresh() override
    {
        yawServo.updateSendPwmRamp();
        pitchServo.updateSendPwmRamp();
    }

    void refreshSafeDisconnect() override
    {
        mcbLite.pwm.write(0.0f, yawServo.getPin());
        mcbLite.pwm.write(0.0f, pitchServo.getPin());
    }

    void moveToCube(const tap::algorithms::transforms::Transform& vtmGimbalToTarget)
    {
        float yaw = vtmGimbalToTarget.getYaw();
        float pitch = vtmGimbalToTarget.getPitch();

        // uh interpolate from angle to pwm
        float yawPwm = 0.0f;
        float pitchPwm = 0.0f;

        yawServo.setTargetPwm(yawPwm);
        pitchServo.setTargetPwm(pitchPwm);
    }

    aruwsrc::communication::mcb_lite::motor::VirtualServo yawServo;
    aruwsrc::communication::mcb_lite::motor::VirtualServo pitchServo;
    aruwsrc::communication::mcb_lite::MCBLite& mcbLite;

    // figure this out
    static constexpr float YAW_MIN_PWM = 0.00f;
    static constexpr float YAW_MAX_PWM = 0.00f;
    static constexpr float YAW_MIN_ANGLE = -M_PI_2; // im assuming its not 360 
    static constexpr float YAW_MAX_ANGLE = M_PI_2;

    static constexpr float PITCH_MIN_PWM = 0.00f;
    static constexpr float PITCH_MAX_PWM = 0.00f;
    static constexpr float PITCH_MIN_ANGLE = -M_PI_2; // same here
    static constexpr float PITCH_MAX_ANGLE = M_PI_2;

    static constexpr float RAMP_SPEED = 0.01f; 

private:
    const aruwsrc::engineer::algorithms::EngineerTransforms& transforms;
};
}  // namespace aruwsrc::engineer
#endif