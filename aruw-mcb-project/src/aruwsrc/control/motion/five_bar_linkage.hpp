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

#include "tap/algorithms/smooth_pid.hpp"
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
};

class FiveBarLinkage
{
public:
    FiveBarLinkage(
        tap::Drivers* drivers,
        tap::motor::MotorInterface* motor1,
        tap::motor::MotorInterface* motor2,
        FiveBarConfig fiveBarConfig,
        tap::algorithms::SmoothPidConfig motorPidConfig);

    void initialize();

    inline void setDesiredPosition(modm::Vector2f desiredPosition){
        this->desiredPosition = desiredPosition;
    };

    modm::Vector2f getDesiredPosition() { return desiredPosition; };

    modm::Location2D<float> getCurrentPosition() { return currentPosition; };

    FiveBarConfig getFiveBarConfig() { return fiveBarConfig; };

    void refresh();

private:
    /**
     * Position of the five bar linkage relative to the centerpoint between the two motors.
     * X+ = from motor 2 to motor 1
     * Y+ = to the left of x+
     * orientation = angle of motor link (x+ side)
     *      (M2)  y+ ^     (M1)
     *          _____|______         x+ ->
     *         /           \
     *      ./              \.
     *       \              /
     *        \           /
     *          \       / (orientation)
     *            \   /
     *              O
     */
    modm::Location2D<float> currentPosition;

    /**
     * Desired position. See sign convention above     *
     */
    modm::Vector2f desiredPosition;

    FiveBarConfig fiveBarConfig;

    tap::algorithms::SmoothPidConfig motorPidConfig;

    tap::motor::MotorInterface* motor1;
    tap::motor::MotorInterface* motor2;

    tap::algorithms::SmoothPid motor1Pid;
    tap::algorithms::SmoothPid motor2Pid;

    uint32_t prevTime = 0;

    float motor1Setpoint;
    float motor2Setpoint;

    void moveMotors();

    void computeMotorAngles();

};  // class FiveBarLinkageSubsystem
}  // namespace aruwsrc::control::motion

#endif  // FIVE BAR LINKAGE SUBSYSTEM
