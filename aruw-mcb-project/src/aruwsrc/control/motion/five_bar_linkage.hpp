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

#ifndef FIVE_BAR_LINKAGE_HPP_
#define FIVE_BAR_LINKAGE_HPP_

#include "tap/drivers.hpp"
#include "tap/errors/create_errors.hpp"
#include "tap/motor/motor_interface.hpp"

#include "modm/math/geometry/location_2d.hpp"

namespace aruwsrc::control::motion
{
struct FiveBarConfig
{
    modm::Vector2f defaultPosition;
    /***
     * All lengths in Meters, see below for where motor1 and motor2 are.
     */
    float motor1toMotor2Length;
    float motor1toJoint1Length;
    float motor2toJoint2Length;
    float joint1toTipLength;
    float joint2toTipLength;
    /**
     *  minimum and maximum angles for the motors. See below doc for sign convention.
     */
    float motor1MinAngle;
    float motor1MaxAngle;
    float motor2MinAngle;
    float motor2MaxAngle;
};

class FiveBarLinkage
{
public:
    FiveBarLinkage(
        tap::motor::MotorInterface* motor1,
        tap::motor::MotorInterface* motor2,
        FiveBarConfig fiveBarConfig);

    void initialize();

    inline void setDesiredPosition(modm::Vector2f desiredPosition)
    {
        this->desiredPosition = desiredPosition;
    };

    inline void setDesiredPosition(float x, float y)
    {
        this->desiredPosition = modm::Vector2f(x, y);
    };

    modm::Vector2f getDesiredPosition() { return desiredPosition; };

    modm::Vector2f getDefaultPosition() { return fiveBarConfig.defaultPosition; };

    modm::Location2D<float> getCurrentPosition() { return currentPosition; };

    /**
     * Gets the difference between the current position and the desired position of the motor (rad).
     * This function automatically bounds the error such that the error becomes zero when the motor
     * is at the limits definde in the FiveBarConfig
     */
    float getMotor1Error();
    float getMotor2Error();

    float getMotor1RelativePosition() { return motor1RelativePosition; };
    float getMotor2RelativePosition() { return motor2RelativePosition; };

    FiveBarConfig getFiveBarConfig() { return fiveBarConfig; };

    tap::motor::MotorInterface* getMotor1() { return motor1; };

    tap::motor::MotorInterface* getMotor2() { return motor2; };

    void refresh();

    void moveMotors(float motor1output, float motor2output);

private:
    /**
     * Position of the five bar linkage relative to the centerpoint between the two motors.
     * X+ = from motor 2 to motor 1
     * Y+ = to the left of x+
     * orientation = angle of motor link (x+ side)
     *      (M2)  y+ ^  (M1)
     *          ____|____         x+ ->
     *         /         \
     *        /           \
     *       /             \
     *      O               O
     *       \             / (orientation)
     *        \           /
     *         \         /
     *          \       /
     *           \     /
     *            \   /
     *              O
     */
    modm::Location2D<float> currentPosition;

    /**
     * Desired position. See sign convention above.
     */
    modm::Vector2f desiredPosition;

    tap::motor::MotorInterface* motor1;
    tap::motor::MotorInterface* motor2;

    float motor1home;
    float motor2home;
    bool motor1homed = false;
    bool motor2homed = false;

    float motor1RelativePosition;
    float motor2RelativePosition;

    FiveBarConfig fiveBarConfig;

    uint32_t prevTime = 0;

    float motor1Setpoint;
    float motor2Setpoint;

    bool withinEnvelope(modm::Vector2f point);

    int motorsMoved = 0;

    float debug1 = 0;
    float debug2 = 0;
    float debug3 = 0;

    /***
     * There's some magic numbers going on here
     * do the math if you want to
     */
    void computeMotorAngles();

    /**
     * Forward Kinematic Solution. Updates currentPosition based on actual motor1 and motor2 angles.
     */
    void computePositionFromAngles();

};  // class FiveBarLinkageSubsystem
}  // namespace aruwsrc::control::motion

#endif  // FIVE BAR LINKAGE SUBSYSTEM
