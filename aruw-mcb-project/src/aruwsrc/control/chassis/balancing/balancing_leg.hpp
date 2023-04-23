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
    /**
     * @param[in] drivers pointer
     * @param[in] fivebar instantiated in robot_control.
     * See FiveBarLinkage class for definitions of Motor1 and Motor2.
     * @param[in] fivebarMotor1PidConfig
     * @param[in] fivebarMotor1FuzzyPDconfig
     * @param[in] fivebarMotor2PidConfig
     * @param[in] fivebarMotor2FuzzyPDconfig
     * @param[in] wheelMotor instantiated in robot_control
     * @param[in] driveWheelPidConfig
    */
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
     * @param[in] height in mm for the leg to maintain.
     */
    inline void setDesiredHeight(float height) { zDesired = height; };

    /**
     * @param[in] angle of the chassis relative to the wheel to maintain.
     */
    inline void setChassisAngle(float angle) { chassisAngle = angle; };

    /**
     * @param[in] speed of the chassis to maintain.
     */
    inline void setDesiredTranslationSpeed(float speed) { vDesired = speed; }

    /**
     * @return leg height in mm.
     */
    inline float getCurrentHeight() { return zCurrent; };

    /**
     * @return chassis speed in m/s.
     */
    inline float getCurrentTranslationSpeed() { return vCurrent; };

    /**
     * @return FiveBarLinkage instance used by the leg.
     */
    inline aruwsrc::control::motion::FiveBarLinkage* getFiveBar() { return fivebar; };

    /**
     * @return the stand-by position of the fivebar.
     */
    inline modm::Vector2f getDefaultPosition() { return fivebar->getDefaultPosition(); }

    /**
     * Updates the state and control action of the balancing leg.
     */
    void update();

private:
    void computeState(uint32_t dt);

    /**
     * Applies gravity compensation and runs the five-bar linkage controller.
    */
    void fiveBarController(uint32_t dt);

    const float WHEEL_RADIUS;   // (m) radius of the drive wheel

    /* Pointers to required actuators */

    aruwsrc::control::motion::FiveBarLinkage* fivebar;
    tap::motor::MotorInterface* driveWheel;

    /* PID Controllers for actuators */

    tap::algorithms::FuzzyPD fiveBarMotor1Pid;
    tap::algorithms::FuzzyPD fiveBarMotor2Pid;
    tap::algorithms::SmoothPid driveWheelPid;

    /**
     * PID which relates desired x velocity to x positional offset of the wheel which drives x
     * acceleration through the plant. PID loop is essentially used to smoothly move x.
     * output units are m
     */
    tap::algorithms::SmoothPidConfig xPidConfig{
        .kp = .1,
        .ki = 4e-7,
        .kd = 0,
        .maxICumulative = .05,
        .maxOutput = .07,
    };
    tap::algorithms::SmoothPid xPid = tap::algorithms::SmoothPid(xPidConfig);

    tap::algorithms::SmoothPidConfig thetaLPidConfig{
        .kp = 25,
        .ki = 0,
        .kd = 0,
        .maxICumulative = 2,
        .maxOutput = 40,
        .tRProportionalKalman = .5,
    };

    tap::algorithms::SmoothPidConfig thetaLdotPidConfig{
        .kp = 8,
        .ki = 0,
        .kd = 0,
        .maxOutput = 40,
        .tRProportionalKalman = .5,
    };

    tap::algorithms::SmoothPid thetaLPid = tap::algorithms::SmoothPid(thetaLPidConfig);
    
    tap::algorithms::SmoothPid thetaLdotPid = tap::algorithms::SmoothPid(thetaLdotPidConfig);

    uint32_t prevTime = 0;

    // debugging variables for when I'm too much of a bitch to turn the robot on
    float xdes = 0;
    float iwdes = 0;
    float debug1;
    float debug2;
    float debug3;

    float zDesired,     // (m) world-frame height of the chassis
        zCurrent;       // (m)

    float vDesired;     // (m/s) world-frame leg speed in the x-direciton
    float vCurrent;     // (m/s)
    float vCurrentPrev; // (m/s)

    float chassisAngle,     // (rad) positive-down pitch angle of the chassis
    chassisAnglePrev;       // (rad)
    float chassisAngledot;  // (rad/s)

    float motorLinkAnglePrev;   // (rad) angle of the link that the wheel motor is attached to for offsetting

    // TODO: Derek WTF are these
    std::array<float, 10> tlWindow;
    uint8_t tlWindowIndex = 0;
    float tl_dot_w, tl_ddot_w;

    float tl,          // (rad) angle of the metaphyiscal pendulum
        tl_prev,       // (rad)
        tl_prev_prev,  // (rad)
        tl_dot,        // (rad/s)
        tl_dotPrev,    // (rad/s)
        tl_ddot,       // (rad/s^2)
        tl_ddotPrev;   // (rad/s^2)

    float realWheelSpeed;    // (rad/s) rotation of wheel, +rotation = forward motion
    float wheelPosPrev = 0;  // (rad) just used to compute the wheel's speed

    float aCurrent,    // (m/s^2) rate of change of vCurrent
        aCurrentPrev,  // (m/s^2)
        aDesired,      // (m/s^2)
        aDesiredPrev;  // (m/s^2)

    float xoffset,      // (m) x-offset used to drive linear acceleration
        xoffsetPrev;    // (m)
};
}  // namespace chassis
}  // namespace aruwsrc

#endif  // BALANCING_LEG_HPP_
