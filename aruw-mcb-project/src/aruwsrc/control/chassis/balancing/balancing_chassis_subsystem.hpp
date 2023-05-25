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
#include "tap/algorithms/transforms/transform.hpp"
#include "tap/communication/sensors/current/analog_current_sensor.hpp"
#include "tap/control/chassis/chassis_subsystem_interface.hpp"
#include "tap/control/chassis/power_limiter.hpp"
#include "tap/control/subsystem.hpp"

#include "aruwsrc/communication/can/turret_mcb_can_comm.hpp"
#include "aruwsrc/control/chassis/constants/chassis_constants.hpp"
#include "aruwsrc/control/turret/turret_motor.hpp"
#include "aruwsrc/robot/balstd/balstd_frames.hpp"

#include "balancing_leg.hpp"

using namespace aruwsrc::balstd::transforms;

namespace aruwsrc::chassis
{

enum LegHomingState
{
    UNHOMED = 0,
    HOMING,
    HOMED
};

class BalancingChassisSubsystem : public tap::control::chassis::ChassisSubsystemInterface
{
public:
    BalancingChassisSubsystem(
        tap::Drivers* drivers,
        aruwsrc::can::TurretMCBCanComm& turretMCB,
        const aruwsrc::control::turret::TurretMotor& pitchMotor,
        const aruwsrc::control::turret::TurretMotor& yawMotor,
        BalancingLeg& leftLeg,
        BalancingLeg& rightLeg,
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
        homingState = HOMING;
        legHomingPid[0].reset();
        legHomingPid[1].reset();
        legHomingPid[2].reset();
        legHomingPid[3].reset();
        legStalled[0] = legStalled[1] = legStalled[2] = legStalled[3] = false;
    };

    /**
     * @return the number of chassis motors
     */
    inline int getNumChassisMotors() const override { return 2; };

    /**
     * @return `true` iff all motors are online
     */
    inline bool allMotorsOnline() const override
    {
        return leftLeg.wheelMotorOnline() && rightLeg.wheelMotorOnline();
    };

    /**
     * Retracts legs to home position and kills drive motors. Leg motors are still active to get to
     * and hold position.
     */
    void stopChassis()
    {
        leftLeg.getFiveBar()->setDesiredPosition(leftLeg.getDefaultPosition());
        rightLeg.getFiveBar()->setDesiredPosition(rightLeg.getDefaultPosition());
        setDesiredOutput(0, 0);
        disarmChassis();
    };

    inline void setFallen()
    {
        leftLeg.setBalancingState(FALLEN_MOVING);
        rightLeg.setBalancingState(FALLEN_MOVING);
    }

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
        desiredZ = tap::algorithms::limitVal<float>(desiredZ + z, -.35, -.1);
    };

    BalancingLeg& getLeftLeg() { return leftLeg; };
    BalancingLeg& getRightLeg() { return rightLeg; };

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

    inline void armChassis()
    {
        armed = true;
        if (!rightLeg.getArmState()) rightLeg.armLeg();
        if (!leftLeg.getArmState()) leftLeg.armLeg();
    };
    inline void disarmChassis()
    {
        armed = false;
        if (rightLeg.getArmState()) rightLeg.disarmLeg();
        if (leftLeg.getArmState()) leftLeg.disarmLeg();
    };
    inline void toggleArm() { armed ? disarmChassis() : armChassis(); };
    inline bool getArmState() { return armed; };

    tap::algorithms::SmoothPid rotationPid;

    LegHomingState homingState = HOMED;

private:
    tap::algorithms::transforms::Transform<Chassis, Turret> chassisToTurret =
        tap::algorithms::transforms::Transform<Chassis, Turret>();

    bool armed = false;

    tap::algorithms::Ramp velocityRamper;
    static constexpr float MAX_ACCELERATION = .5;  // m/s/s

    void computeState(uint32_t dt);

    void getAngles(uint32_t dt);

    void homeLegs(uint32_t dt);

    void updateLegOdometry();

    aruwsrc::can::TurretMCBCanComm& turretMCB;
    const aruwsrc::control::turret::TurretMotor& pitchMotor;
    const aruwsrc::control::turret::TurretMotor& yawMotor;

    tap::communication::sensors::current::AnalogCurrentSensor currentSensor;

    tap::control::chassis::PowerLimiter chassisPowerLimiter;
    BalancingLeg &leftLeg, rightLeg;

    static modm::Pair<int, float> lastComputedMaxWheelSpeed;

    SmoothPidConfig rollPidConfig{
        .kp = .01,
        .ki = .00002,
        .kd = .001,
        .maxICumulative = .1,
        .maxOutput = .1,
    };
    SmoothPid rollPid = SmoothPid(rollPidConfig);

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

    float debug1;
    float debug2;
    float debug3;
    float debug4;

    float pitchAdjustment = 0;
    float pitchAdjustmentPrev = 0;
    float velocityAdjustment = 0;
    float velocityAdjustmentPrev = 0;
    float targetPitch;

    /**
     * @brief Floating point world-relative orientation information. Angles are in rad, Angular Rate
     * in rad/s. Prev values are used for finite-differencing and low-pass-filtering.
     *
     */
    float pitch;
    float pitchPrev;
    float pitchRate;
    float roll;
    float rollPrev;
    float rollRate;
    float yaw;
    float yawPrev;
    float yawRate;

    float desiredX, desiredV, desiredR, desiredZ;
    float currentX, currentV, currentR, currentZ;
    float prevX, prevV, prevR, prevZ;
    float prevXdesired, prevVdesired;
    uint32_t prevTime;
};
}  // namespace aruwsrc::chassis

#endif  // BALANCING_CHASSIS_SUBSYSTEM_HPP_
