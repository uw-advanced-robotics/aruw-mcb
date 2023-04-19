/*
 * Copyright (c) 2020-2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef BALANCING_LEG_HPP_
#define BALANCING_LEG_HPP_

#include "tap/algorithms/fuzzy_pd.hpp"
#include "tap/motor/dji_motor.hpp"

#include "aruwsrc/control/motion/five_bar_linkage.hpp"
#include "aruwsrc/motor/tmotor_ak80-9.hpp"

namespace aruwsrc
{
namespace chassis
{
/**
 * Wrapper class for a mechanism that uses a five-bar linkage
 * with links grounded on a robot chassis and a wheel motor
 * attached to the end-effector to drive one side of a two-wheeled robot.
 *
 * See @class BalancingChassisSubsystem for application of this mechanism.
 */
class BalancingLeg
{
public:
    BalancingLeg(
        tap::Drivers* drivers,
        aruwsrc::control::motion::FiveBarLinkage* fivebar,
        const tap::algorithms::SmoothPidConfig fivebarMotor1PidConfig,
        const tap::algorithms::FuzzyPDConfig fivebarMotor1FuzzyPDconfig,
        const tap::algorithms::SmoothPidConfig fivebarMotor2PidConfig,
        const tap::algorithms::FuzzyPDConfig fivebarMotor2FuzzyPDconfig,
        tap::motor::MotorInterface* wheelMotor,
        const float wheelRadius,
        const tap::algorithms::SmoothPidConfig driveWheelPidConfig);

    void initialize();

    /**
     *
     */
    inline void setDesiredHeight(float height) { zDesired = height; };

    inline void setChassisAngle(float angle) { chassisAngle = angle; };

    /**
     *
     */
    inline void setDesiredTranslationSpeed(float speed) { vDesired = speed; }

    /**
     *
     */
    inline float getCurrentHeight() { return zCurrent; };

    /**
     *
     */
    inline float getCurrentTranslationSpeed() { return vCurrent; };

    inline aruwsrc::control::motion::FiveBarLinkage* getFiveBar() { return fivebar; };

    inline modm::Vector2f getDefaultPosition() { return fivebar->getDefaultPosition(); }

    /**
     *
     */
    void update();

private:
    void computeState(uint32_t dt);

    void fiveBarController(uint32_t dt);

    const float WHEEL_RADIUS;

    aruwsrc::control::motion::FiveBarLinkage* fivebar;

    tap::algorithms::FuzzyPD fiveBarMotor1Pid;
    tap::algorithms::FuzzyPD fiveBarMotor2Pid;

    tap::motor::MotorInterface* driveWheel;

    tap::algorithms::SmoothPid driveWheelPid;

    /**
     * PID which relates desired x velocity to x positional offset of the wheel which drives x
     * acceleration through the plant. PID loop is essentially used to smoothly move x.
     * output units are m
     *
     */
    tap::algorithms::SmoothPidConfig xPidConfig{
        .kp = .03,
        .ki = 0,
        .kd = 0,
        .maxICumulative = .1,
        .maxOutput = .05,
    };
    tap::algorithms::SmoothPid xPid = tap::algorithms::SmoothPid(xPidConfig);

    tap::algorithms::SmoothPidConfig thetaLPidConfig{
        .kp = 1,
        .ki = 0,
        .kd = 0,
        .maxOutput = 40,
    };

    tap::algorithms::SmoothPid thetaLPid = tap::algorithms::SmoothPid(thetaLPidConfig);

    uint32_t prevTime = 0;

    // debugging variables for when I'm too much of a bitch to turn the robot on
    float xdes = 0;
    float iwdes = 0;
    float debug1;
    float debug2;
    float debug3;

    float zDesired,  // m, world-frame height
        zCurrent;    // m

    float vDesired;      // m/s, world-frame leg speed in x-direction
    float vCurrent;      // m/s
    float vCurrentPrev;  // m/s

    float chassisAngle,     // rad, with positive = pitch down
        chassisAnglePrev;   // rad
    float chassisAngledot;  // rad/s, see chassisAngle

    float motorLinkAnglePrev;  // rad, angle of the link that the wheel motor is attached to for
                               // offsetting

    std::array<float, 10> tlWindow;
    uint8_t tlWindowIndex = 0;
    float tl_dot_w, tl_ddot_w;

    float tl,          // rad, angle of the metaphyiscal pendulum
        tl_prev,       // rad
        tl_prev_prev,  // rad
        tl_dot,        // rad/s
        tl_dotPrev,    // rad/s
        tl_ddot,       // rad/s/s
        tl_ddotPrev;   // rad/s/s

    float realWheelSpeed;    // rad/s, rotation of wheel, +rotation = forward motion
    float wheelPosPrev = 0;  // rad, just used to compute the wheel's speed

    float aCurrent,    // m/s/s, rate of change of vCurrent
        aCurrentPrev,  // m/s/s
        aDesired,      // m/s/s
        aDesiredPrev;  // m/s/s

    float xoffset,  // m, x-offset used to drive linear acceleration
        xoffsetPrev;
};
}  // namespace chassis
}  // namespace aruwsrc

#endif  // BALANCING_LEG_HPP_
