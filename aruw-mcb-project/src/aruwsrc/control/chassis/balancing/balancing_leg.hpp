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
#include "tap/algorithms/ramp.hpp"
#include "tap/motor/dji_motor.hpp"

#include "aruwsrc/communication/can/turret_mcb_can_comm.hpp"
#include "aruwsrc/control/chassis/constants/chassis_constants.hpp"
#include "aruwsrc/control/motion/five_bar_linkage.hpp"
#include "aruwsrc/motor/tmotor_ak80-9.hpp"

using namespace tap::algorithms;

namespace aruwsrc
{
namespace chassis
{

/**
 * if BALANCING -> run LQR controller for wheel outputs, if it falls, go to FALLEN_MOVING
 * if FALLEN_MOVING -> tell it to stop, if stopped, go to FALLEN_NOT_MOVING
 * if FALLEN_NOT_MOVING && GETUP ENABLE -> go to STANDING UP
 * if STANDING_UP, run feedforward getupper, if it's within unfalling, goto BALANCING
 *
 */
enum BalancingState
{
    BALANCING = 0,
    FALLEN_MOVING,
    FALLEN_NOT_MOVING,
    STANDING_UP,
};

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
     * @param[in] chassisMCB reference to type C on chassis
     * @param[in] fivebar instantiated in robot_control
     * @param[in] fivebarMotor1PidConfig for the left fivebar motor
     * @param[in] fivebarMotor1FuzzyPDconfig for the left fivebar motor
     * @param[in] fivebarMotor2PidConfig for the right fivebar motor
     * @param[in] fivebarMotor2FuzzyPDconfig for the right fivebar motor.
     * @param[in] wheelMotor instantiated in robot_control
     * @param[in] driveWheelPidConfig
     */
    BalancingLeg(
        tap::Drivers* drivers,
        aruwsrc::can::TurretMCBCanComm& chassisMCB,
        aruwsrc::control::motion::FiveBarLinkage* fivebar,
        const SmoothPidConfig fivebarMotor1PidConfig,
        const FuzzyPDConfig fivebarMotor1FuzzyPDconfig,
        const SmoothPidConfig fivebarMotor2PidConfig,
        const FuzzyPDConfig fivebarMotor2FuzzyPDconfig,
        tap::motor::MotorInterface* wheelMotor,
        const SmoothPidConfig driveWheelPidConfig);

    void initialize();

    inline void setBalancingState(BalancingState state) { balancingState = state; }

    inline BalancingState getBalancingState() { return balancingState; }

    /**
     * @param[in] height: (mm) Setpoint for chassis height.
     */
    inline void setDesiredHeight(float height) { zDesired = height; };

    /**
     * @param[in] angle: (rad) Pitch angle to set from external source
     */
    inline void setChassisSpeed(float Speed) { chassisSpeed = Speed; };

    /**
     * @param[in] speed: (m/s) Setpoint for chassis translational speed.
     */
    inline void setDesiredTranslationSpeed(float speed) { vDesired = speed; }

    /**
     * @param[in] yaw: (rad) Setpoint for desired yaw angle, AKA Yaw, relative to where we want to
     * yaw.
     * @param[in] yawRate (rad/s) Current yaw rate
     */
    inline void setChassisYaw(float yaw, float yawRate)
    {
        chassisYaw = yaw;
        chassisYawRate = yawRate;
    }

    /**
     * @brief Sets the average position of the chassis and the desired position (computed from
     * velocity input)
     *
     */
    inline void setChassisPos(float chassispos, float chassisposdesired)
    {
        chassisPos = chassispos;
        chassisPosDesired = chassisposdesired;
    }

    inline void setChassisAngle(float chassisangle, float chassisanglerate)
    {
        chassisAngle = chassisangle;
        chassisAngledot = chassisanglerate;
    }

    /**
     * @brief Get the linear position of the wheel
     *
     * @return (m) position of wheel relative to encoder 0
     */
    inline float getWheelPos() { return wheelPos * WHEEL_RADIUS; }

    /**
     * @return Leg height in mm.
     */
    inline float getCurrentHeight() { return zCurrent; };

    /**
     * @return Chassis speed in m/s.
     */
    inline float getCurrentTranslationSpeed() { return vCurrent; };

    /**
     * @return FiveBarLinkage instance used by the leg.
     */
    inline aruwsrc::control::motion::FiveBarLinkage* getFiveBar() { return fivebar; };

    /**
     * @return The stand-by position of the fivebar.
     */
    inline modm::Vector2f getDefaultPosition() { return fivebar->getDefaultPosition(); }

    /**
     * @return the online-ness of the 3 motors in the balancing leg.
     */
    inline bool wheelMotorOnline() const { return driveWheel->isMotorOnline(); }

    /**
     * Updates the state and control action of the balancing leg.
     */
    void update();

    inline void armLeg() { armed = true; };
    inline void disarmLeg()
    {
        armed = false;
        balancingState = FALLEN_MOVING;
    };
    inline void toggleArm() { armed ? disarmLeg() : armLeg(); };
    inline bool getArmState() { return armed; };

private:
    tap::Drivers* drivers;
    /**
     * Reference to the chassis-mounted Type C MCB
     */
    aruwsrc::can::TurretMCBCanComm& chassisMCB;

    BalancingState balancingState = FALLEN_NOT_MOVING;
    tap::arch::MilliTimeout balanceAttemptTimeout;
    uint32_t BALANCE_ATTEMPT_TIMEOUT_DURATION = 300;
    bool standupEnable = true;
    float STANDUP_TORQUE_GAIN = 1.1;

    /**
     * @param[in] dt (us)
     */
    void computeState(uint32_t dt);

    void updateBalancing(uint32_t dt);
    void updateFallenMoving(uint32_t dt);
    void updateFallenNotMoving(uint32_t dt);
    void updateStandingUp(uint32_t dt);

    void setLegsRetracted();
    /**
     * @brief Heuristic to determing if the robot is in a fallen state, with hysteresis to avoid
     * oscillation. Sets the `isFallen` flag accordingly.
     *
     */
    void iveFallenAndICantGetUp();

    /// Runs control logic with gravity compensation for the five-bar linkage
    void fivebarController(uint32_t dt);

    /* Pointers to required actuators */

    aruwsrc::control::motion::FiveBarLinkage* fivebar;
    tap::motor::MotorInterface* driveWheel;

    /* PID Controllers for actuators */

    FuzzyPD fivebarMotor1Pid;
    FuzzyPD fivebarMotor2Pid;
    SmoothPid driveWheelPid;

    /**
     * Ramps desired height to avoid big step inputs which may cause cringe such as undesired
     * airborneness. To get desired airborneness, override the value.
     *
     */
    tap::algorithms::Ramp zDesRamper;
    // m/s
    static constexpr float Z_RAMP_RATE = .2;
    /**
     * PID which relates desired x velocity to x positional offset of the wheel which drives x
     * acceleration through the plant. PID loop is essentially used to smoothly move x.
     * output units are m
     */
    SmoothPidConfig xPidConfig{
        .kp = 0.003,
        .ki = 2.5e-08,
        .kd = 0,
        .maxICumulative = .02,
        .maxOutput = .07,
    };
    SmoothPid xPid = SmoothPid(xPidConfig);

    uint32_t prevTime = 0;

    bool armed = false;

    // debugging variables for when I'm too much of a bitch to turn the robot on
    float xdes = 0;
    float iwdes = 0;
    float debug1;
    float debug2;
    float debug3;
    float debug4;
    float debug5;
    float debug6;
    float debug7;

    float zDesired,                                       // (m) world-frame height of the chassis
        zCurrent = fivebar->getDefaultPosition().getY();  // (m)

    modm::Vector2f desiredWheelLocation;

    float vDesired;  // (m/s) world-frame leg speed in the x-direciton
    float vDesiredPrev;
    float vCurrent;      // (m/s)
    float vCurrentPrev;  // (m/s)

    float chassisPos;  // (m) Position of chassis (avg of both legs)
    float chassisPosDesired;
    float chassisSpeed;  // (m/s) Total speed of chassis (avg of both legs)
    float chassisYaw;
    float chassisYawRate;

    float chassisAngle,     // (rad) positive-down pitch angle of the chassis
        chassisAnglePrev;   // (rad)
    float chassisAngledot;  // (rad/s)

    float motorLinkAnglePrev;  // (rad) angle of the link that the wheel motor is attached to for
                               // offsetting

    float tl,          // (rad) angle of the metaphyiscal pendulum
        tl_prev,       // (rad)
        tl_prev_prev,  // (rad)
        tl_dot,        // (rad/s)
        tl_dotPrev,    // (rad/s)
        tl_ddot,       // (rad/s^2)
        tl_ddotPrev;   // (rad/s^2)

    float realWheelSpeed;      // (rad/s) rotation of wheel, +rotation = forward motion
    float realWheelSpeedPrev;  // (rad/s) Used for LPF
    float wheelPosDesired;     // (rad)
    float wheelPos;
    float wheelPosPrev;  // (rad) just used to compute the wheel's speed

    float aCurrent,  // (m/s^2) rate of change of vCurrent
        aCurrentPrev,
        aDesired,      // (m/s^2)
        aDesiredPrev;  // (m/s^2)
    float aChassis;    // (m/s^2)

    float xoffset,    // (m) x-offset used to drive linear acceleration
        xoffsetPrev;  // (m)
    float prevOutput;

    static constexpr float FALLEN_ANGLE_THRESHOLD = modm::toRadian(25);
    static constexpr float FALLEN_ANGLE_RETURN = modm::toRadian(3);
    static constexpr float FALLEN_ANGLE_RATE_THRESHOLD = 4;
};
}  // namespace chassis
}  // namespace aruwsrc

#endif  // BALANCING_LEG_HPP_
