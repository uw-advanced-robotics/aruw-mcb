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

/**
 * Sign convention for angles is z+ with 0 at the x-axis.
 *
 * See FiveBarLinkage class for diagram and further details.
 */
struct FiveBarConfig
{
    modm::Vector2f defaultPosition;

    float motor1toMotor2Length;  // (m)
    float motor1toJoint1Length;  // (m)
    float motor2toJoint2Length;  // (m)
    float joint1toTipLength;     // (m)
    float joint2toTipLength;     // (m)

    float motor1MinAngle;  // (rad)
    float motor1MaxAngle;  // (rad)
    float motor2MinAngle;  // (rad)
    float motor2MaxAngle;  // (rad)
};

/**
 * Wrapper class for a five-bar linkage object.
 *
 * COORDINATE SYSTEM CONVENTION
 *
 * X+ = from motor 1 to motor 2
 * Y+ = to the right of x+
 * orientation = angle of motor link (x+ side)
 *      (M2)  y- ^  (M1)
 *          ____|____         x- ->
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

    inline void setHomePosition() {}

    modm::Vector2f getDesiredPosition() { return desiredPosition; };

    modm::Vector2f getDefaultPosition() { return fiveBarConfig.defaultPosition; };

    modm::Location2D<float> getCurrentPosition() { return currentPosition; };

    float getCurrentLength() { return currentL; };
    float getCurrentAngle() { return currentTheta; };

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
    modm::Location2D<float> currentPosition;  // (m) for position, (rad) for orientation
    // represent end effector position in polar, updated from currenposition
    float currentTheta;
    float currentL;
    modm::Vector2f desiredPosition;  // (m)

    tap::motor::MotorInterface* motor1;
    tap::motor::MotorInterface* motor2;

    float motor1Home;  // (rad)
    float motor2Home;  // (rad)
    bool motor1IsHomed = false;
    bool motor2IsHomed = false;

    float motor1RelativePosition;  // (rad)
    float motor2RelativePosition;  // (rad)

    FiveBarConfig fiveBarConfig;

    uint32_t prevTime = 0;

    float motor1Setpoint;  // (rad)
    float motor2Setpoint;  // (rad)

    /***
     * Converts setpoints from XY-coordinates to motor angles using inverse kinematics.
     */
    void computeMotorAngleSetpoints();

    /**
     * Forward Kinematic Solution.
     * Updates currentPosition based on actual motor1 and motor2 angles using a look-up table.
     */
    void computePositionFromAngles();

    float debug1;
    float debug2;
    float debug3;
    float debug4;

};  // class FiveBarLinkageSubsystem
}  // namespace aruwsrc::control::motion

#endif  // FIVE BAR LINKAGE SUBSYSTEM
