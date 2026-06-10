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

#include "engineer_servo_constants.hpp"

namespace aruwsrc::engineer::servo
{
class VTMServoSubsystem : public tap::control::Subsystem
{
public:
    VTMServoSubsystem(
        tap::Drivers* drivers,
        aruwsrc::communication::mcb_lite::motor::VirtualServo& yawServo,
        aruwsrc::communication::mcb_lite::motor::VirtualServo& pitchServo,
        aruwsrc::communication::mcb_lite::MCBLite& mcbLite)
        : tap::control::Subsystem(drivers),
          yawServo(yawServo),
          pitchServo(pitchServo),
          mcbLite(mcbLite)
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
        float x = vtmGimbalToTarget.getX();
        float y = vtmGimbalToTarget.getY();
        float z = vtmGimbalToTarget.getZ();

        // conversion?
        float yaw = atan2f(y, x);
        float pitch = atan2f(-z, sqrtf(x * x + y * y));

        // convert from angle to pwm
        float yawPwm = YAW_MIN_PWM + (yaw - YAW_MIN_ANGLE) / (YAW_MAX_ANGLE - YAW_MIN_ANGLE) *
                                         (YAW_MAX_PWM - YAW_MIN_PWM);
        float pitchPwm = PITCH_MIN_PWM + (pitch - PITCH_MIN_ANGLE) /
                                             (PITCH_MAX_ANGLE - PITCH_MIN_ANGLE) *
                                             (PITCH_MAX_PWM - PITCH_MIN_PWM);

        yawServo.setTargetPwm(yawPwm);
        pitchServo.setTargetPwm(pitchPwm);
    }

    aruwsrc::communication::mcb_lite::motor::VirtualServo& yawServo;
    aruwsrc::communication::mcb_lite::motor::VirtualServo& pitchServo;
    aruwsrc::communication::mcb_lite::MCBLite& mcbLite;

private:
};
}  // namespace aruwsrc::engineer::servo
#endif