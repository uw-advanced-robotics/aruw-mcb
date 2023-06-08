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

#ifndef BALANCING_CHASSIS_SUBSYSTEM_HPP_
#define BALANCING_CHASSIS_SUBSYSTEM_HPP_

#include "tap/algorithms/ramp.hpp"
#include "tap/algorithms/smooth_pid.hpp"
#include "tap/algorithms/transforms/transform.hpp"
#include "tap/communication/sensors/current/analog_current_sensor.hpp"
#include "tap/control/chassis/chassis_subsystem_interface.hpp"
#include "tap/control/chassis/power_limiter.hpp"
#include "tap/control/subsystem.hpp"

#include "aruwsrc/communication/can/turret_mcb_can_comm.hpp"
#include "aruwsrc/control/chassis/constants/chassis_constants.hpp"
#include "aruwsrc/control/turret/turret_motor.hpp"
#include "aruwsrc/motor/tmotor_ak80-9.hpp"
#include "aruwsrc/robot/balstd/balstd_frames.hpp"

using namespace aruwsrc::balstd::transforms;
using namespace tap::algorithms;

namespace aruwsrc::chassis
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
    JUMPING,
    FALLING,
    HOMING,
    RECOVERY
};

class BalancingChassisSubsystem : public tap::control::chassis::ChassisSubsystemInterface
{
public:
    BalancingChassisSubsystem(
        tap::Drivers* drivers,
        aruwsrc::can::TurretMCBCanComm& turretMCB,
        const aruwsrc::control::turret::TurretMotor& pitchMotor,
        const aruwsrc::control::turret::TurretMotor& yawMotor,
        aruwsrc::control::motion::FiveBarLinkage* fivebarLeft,
        aruwsrc::control::motion::FiveBarLinkage* fivebarRight,
        tap::motor::MotorInterface* driveWheelLeft,
        tap::motor::MotorInterface* driveWheelRight,
        tap::gpio::Analog::Pin currentPin = CURRENT_SENSOR_PIN);

    void initialize() override;

    void refresh() override;

    void runHardwareTests() override;

    const char* getName() override { return "Balancing Chassis Subsystem"; }

    /**
     * Initializes the leg motor homing procedure, where it draws all 4 legs to the upper hardstops.
     *
     */
    void startHoming()
    {
        balancingState = HOMING;
        legHomingPid[0].reset();
        legHomingPid[1].reset();
        legHomingPid[2].reset();
        legHomingPid[3].reset();
        legStalled[0] = legStalled[1] = legStalled[2] = legStalled[3] = false;
    };

    void startJumping() {}

    /**
     * @return the number of chassis motors
     */
    inline int getNumChassisMotors() const override { return 2; };

    /**
     * @return `true` iff all motors are online
     */
    inline bool allMotorsOnline() const override
    {
        return driveWheelLeft->isMotorOnline() && driveWheelRight->isMotorOnline();
    };

    /**
     * Retracts legs to home position and kills drive motors. Leg motors are still active to get to
     * and hold position.
     */
    void stopChassis()
    {
        setSafeBehavior();
        setDesiredOutput(0, 0);
        disarmChassis();
    };

    /**
     * Changes setpoints to safe behavior,
     */
    void setSafeBehavior();

    inline void setFallen() { balancingState = FALLEN_MOVING; }

    /**
     * @return The actual chassis velocity in chassis relative frame, as a vector <vx, vy, vz>,
     *      where vz is rotational velocity. This is the velocity calculated from the chassis's
     *      encoders. Units: m/s
     */
    modm::Matrix<float, 3, 1> getActualVelocityChassisRelative() const
    {
        const float data[3] = {currentV, 0, currentR};
        return modm::Matrix<float, 3, 1>(data);
    };

    /**
     * @brief Get the Chassis Orientation relative to the chassis orientation
     *
     * @return roll-pitch-yaw
     */
    inline modm::Matrix<float, 3, 1> getChassisOrientationWorldRelative() const
    {
        const float data[3] = {roll, pitch, yaw};
        return modm::Matrix<float, 3, 1>(data);
    };

    /**
     * @brief Get the Chassis Orientation Rates relative to a fixed world orientation
     *
     * @return roll-pitch-yaw derivatives
     */
    inline modm::Matrix<float, 3, 1> getChassisOrientationRates() const
    {
        const float data[3] = {rollRate, pitchRate, yawRate};
        return modm::Matrix<float, 3, 1>(data);
    };

    void setDesiredHeight(float z)
    {
        desiredZRamper.setTarget(tap::algorithms::limitVal<float>(
            desiredZRamper.getTarget() - z,
            CHASSIS_HEIGHTS.first,
            CHASSIS_HEIGHTS.second));
    };

    /**
     * @brief Defines x position and r rotation angle that we want the chassis to be in
     *
     * @param v: (m/s) Velocity Setpoint Relative to current position
     * @param r: (rad) Yaw Setpoint relative to current Yaw
     */
    void setDesiredOutput(float v, float r)
    {
        velocityRamper.setTarget(v);
        desiredR = r;
    };

    void limitChassisPower();

    static inline float getMaxWheelSpeed(bool refSerialOnline, int chassisPower)
    {
        static modm::Pair<int, float> lastComputedMaxWheelSpeed = CHASSIS_POWER_TO_MAX_SPEED_LUT[0];
        if (!refSerialOnline)
        {
            chassisPower = 0;
        }

        // only re-interpolate when needed (since this function is called a lot and the chassis
        // power rarely changes, this helps cut down on unnecessary array searching/interpolation)
        if (lastComputedMaxWheelSpeed.first != chassisPower)
        {
            lastComputedMaxWheelSpeed.first = chassisPower;
            lastComputedMaxWheelSpeed.second =
                CHASSIS_POWER_TO_SPEED_INTERPOLATOR.interpolate(chassisPower);
        }

        return lastComputedMaxWheelSpeed.second;
    }

    inline void armChassis() { armed = true; };
    inline void disarmChassis() { armed = false; };
    inline void toggleArm() { armed ? disarmChassis() : armChassis(); };
    inline bool getArmState() { return armed; };

    inline bool getStandEnabled() { return standupEnable; }
    inline void enableStand() { standupEnable = true; }
    inline void disableStand() { standupEnable = false; }

    inline BalancingState getBalancingState() { return balancingState; }

    tap::algorithms::SmoothPid rotationPid;

private:
    tap::algorithms::transforms::Transform<Chassis, Turret> chassisToTurret =
        tap::algorithms::transforms::Transform<Chassis, Turret>();

    bool armed = false;

    tap::algorithms::Ramp velocityRamper;
    static constexpr float MAX_ACCELERATION = .5;  // m/s/s

    // Tunable constants related to the state transitions
    BalancingState balancingState = FALLEN_NOT_MOVING;
    tap::arch::MilliTimeout balanceAttemptTimeout;
    uint32_t BALANCE_ATTEMPT_TIMEOUT_DURATION = 300;
    bool standupEnable = false;
    float STANDUP_TORQUE_GAIN = 1.1;

    float FALLING_FORCE_THRESHOLD = 40.0;  // Newtons
    float JUMP_GRAV_GAIN = 1.0f;
    tap::arch::MilliTimeout jumpTimeout;
    uint32_t JUMP_TIMEOUT_DURATION = 200;

    static constexpr float FALLEN_ANGLE_THRESHOLD = modm::toRadian(26);
    static constexpr float FALLEN_ANGLE_RETURN = modm::toRadian(5);
    static constexpr float FALLEN_ANGLE_RATE_THRESHOLD = 3;

    /**
     * Estimates the current state variables of the robot.
     *
     * @param dt, delta time between previous and current cycles in ms
     */
    void computeState(uint32_t dt);

    /**
     * Estimates leg link force based on feedback, and decides if the robot is freefalling
     *
     */
    void airborneDetector();

    /**
     * Conducts rotational transforms between turret/turretMCB to get chassis angles
     *
     * @param dt, delta time between previous and current cycles in ms
     */
    void getAngles(uint32_t dt);

    /**
     * FSM state when the robot is balancing by itself. If the robot disarms, this prevents wheels
     * from outputting.
     */
    void updateBalancing(uint32_t dt);
    /**
     * FSM state for when the robot is fallen down and moving (has nonzero speed across the floor).
     * Waits until the chassis stops to continue.
     */
    void updateFallenMoving(uint32_t dt);
    /**
     * FSM state for when the robot has stopped moving, is fallen, and is ready to get back up.
     * Automatic getting-up is gated by the `standupEnable` flag. If the chassis is sufficiently up
     * as defined by the `FALLEN_ANGLE` parameters, we go to the `BALANCING` state
     */
    void updateFallenNotMoving(uint32_t dt);
    /**
     * FSM state for automatically standing. The system executes a feedforward to the wheels to
     * "kick" the robot up, until it is level enough to automatically balance.
     */
    void updateStandingUp(uint32_t dt);
    /**
     * FSM state for jumping. It executes `updateBalancing` but overrides the z- setpoint to the
     * high position and bypasses the ramper.
     */
    void updateJumping(uint32_t dt);
    /**
     * FSM state for homing the legs. It runs the legs into the upper hardstops until they stop.
     */
    void homeLegs(uint32_t dt);
    /**
     * FSM state for recovery mode. Retracts legs and lets the user manually drive the robot in the
     * unbalanced state to recover the robot if it gets stuck.
     */
    void updateRecovery(uint32_t dt);
    /**
     * FSM state for when the robot is in freefall. It extends the legs straight down in
     * world-frame.
     */
    void updateFalling();

    void setLegsRetracted(uint32_t dt);

    /// Runs control logic with gravity compensation for the five-bar linkage
    void fivebarController();

    aruwsrc::can::TurretMCBCanComm& turretMCB;
    const aruwsrc::control::turret::TurretMotor& pitchMotor;
    const aruwsrc::control::turret::TurretMotor& yawMotor;

    tap::communication::sensors::current::AnalogCurrentSensor currentSensor;

    tap::control::chassis::PowerLimiter chassisPowerLimiter;

    /* Pointers to required actuators */

    aruwsrc::control::motion::FiveBarLinkage* fivebarLeft;
    aruwsrc::control::motion::FiveBarLinkage* fivebarRight;
    tap::motor::MotorInterface* driveWheelLeft;
    tap::motor::MotorInterface* driveWheelRight;

    static modm::Pair<int, float> lastComputedMaxWheelSpeed;

    SmoothPid rollPid = SmoothPid(SmoothPidConfig{
        .kp = .01,
        .kd = .001,
        .maxICumulative = .1,
        .maxOutput = .1,
    });

    SmoothPid heightPid = SmoothPid(SmoothPidConfig{
        .kp = 800,
        .ki = .5,
        .kd = -10000,
        .maxICumulative = 100,
        .maxOutput = 400,
    });

    SmoothPidConfig retractionPidConfig = {
        .kp = 500,
        .ki = .0,
        .kd = .0,
        .maxICumulative = 2,
        .maxOutput = 20,
        .errDeadzone = .01,
    };
    SmoothPid retractionPid[2] = {SmoothPid(retractionPidConfig), SmoothPid(retractionPidConfig)};

    SmoothPidConfig retractionAnglePidConfig = {
        .kp = 5,
        .ki = 0,
        .kd = 0,
        .maxICumulative = 1,
        .maxOutput = 5,
        .errDeadzone = .01,
    };
    SmoothPid retractionAnglePid[2] = {
        SmoothPid(retractionAnglePidConfig),
        SmoothPid(retractionAnglePidConfig)};

    SmoothPid linkAngleMismatchPid = SmoothPid(SmoothPidConfig{
        .kp = 50,
        .kd = 100,
        .maxICumulative = .1,
        .maxOutput = 5,
    });

    SmoothPid yawPid = SmoothPid(SmoothPidConfig{
        .kp = .01,
        .kd = .001,
        .maxICumulative = .1,
        .maxOutput = .1,
    });

    SmoothPidConfig legHomingPidConfig{
        .kp = 10000,
        .ki = 300,
        .kd = 3,
        .maxICumulative = 25000,
        .maxOutput = 25000,
    };
    // LF, LR, RF, RR
    SmoothPid legHomingPid[4] = {
        SmoothPid(legHomingPidConfig),
        SmoothPid(legHomingPidConfig),
        SmoothPid(legHomingPidConfig),
        SmoothPid(legHomingPidConfig)};
    bool legStalled[4] = {false, false, false, false};
    tap::arch::MilliTimeout motorHomedTimeout[4];
    uint32_t STALL_CURRENT = 20000;
    uint32_t STALL_TIMEOUT = 1000;

    /**
     * Ramps desired height to avoid big step inputs which may cause cringe such as undesired
     * airborneness. To get desired airborneness, override the value.
     *
     */
    tap::algorithms::Ramp desiredZRamper;
    // m/s
    static constexpr float Z_RAMP_RATE = .3;

    float debug1;
    float debug2;
    float debug3;
    float debug4;
    float* debugMat1;
    float debugMat2[2];

    /**
     * Floating point world-relative orientation information. Angles are in rad, Angular Rate
     * in rad/s. Prev values are used for finite-differencing and low-pass-filtering.
     */
    float pitch, pitchPrev, pitchRate;
    float tl, tlPrev, tlRate;
    float roll, rollPrev, rollRate;
    float yaw, yawPrev, yawRate;

    float desiredX, desiredV, desiredR, desiredZ;
    float currentX, currentV, currentR, currentZ;
    float prevX, prevV, prevR, prevZ;
    float prevXdesired, prevVdesired;
    uint32_t prevTime;

    // Wheel Position/Velocity in m, m/s
    float wheelPosLeft, wheelPosRight;
    float wheelPosPrevLeft, wheelPosPrevRight;
    float wheelVelLeft, wheelVelRight;

    // States of the individual sides
    float currentZLeft, currentZRight;
    float tlLeft, tlRight;

    // State Variables related to the control system
    float wheelTorqueLeft, wheelTorqueRight;
    float desiredLinkTorqueLeft, desiredLinkTorqueRight;
    float currentLinkForceLeft, currentLinkForceRight;
    float desiredLinkForceLeft, desiredLinkForceRight;
};
}  // namespace aruwsrc::chassis

#endif  // BALANCING_CHASSIS_SUBSYSTEM_HPP_
