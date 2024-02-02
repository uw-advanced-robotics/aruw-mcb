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
#include "mecanum_wheel.hpp"

namespace aruwsrc
{
namespace chassis
{
MecanumWheel::MecanumWheel(Motor& driveMotor, const WheelConfig& config)
    : Wheel(config),
      driveMotor(driveMotor),
      velocityPid(SmoothPid(config.velocityPidConfig))
{
    MAT1 = CMSISMat<2, 2>({0.0f,
                                                (float)sin(WHEEL_RELATIVE_TO_ROLLER_ANGLE),
                                                config.diameter / 2,
                                                (float)cos(WHEEL_RELATIVE_TO_ROLLER_ANGLE)});
    MAT1 = MAT1.inverse();
    MAT2 = CMSISMat<2, 2>({(float)cos(AXLE_TO_ROBOT_FRONT),
                                                (float)-sin(AXLE_TO_ROBOT_FRONT),
                                                (float)sin(AXLE_TO_ROBOT_FRONT),
                                                (float)cos(AXLE_TO_ROBOT_FRONT)});
    MAT2 = MAT2.inverse();
    PRODUCT_MAT = MAT1 * MAT2;   
    PRODUCT_MAT =   CMSISMat<2, 2>({0.0f, 0.0f, 0.0f, 0.0f});                                                     
}

void MecanumWheel::executeWheelVelocity(float vx, float vy)
{
    wheelMat = CMSISMat<2, 1>({vx, vy});
    CMSISMat<2, 1> desiredMat = PRODUCT_MAT * wheelMat;
    driveSetPoint = desiredMat.data[0];
}

void MecanumWheel::initialize()
{
    if (config.isPowered)
    {
        driveMotor.initialize();
    }
}

void MecanumWheel::refresh() {
    if (config.isPowered) {
        driveMotor.setDesiredOutput(velocityPid.runControllerDerivateError(driveSetPoint - driveMotor.getShaftRPM(), 2.0f));
    }
}

void MecanumWheel::setZeroRPM() {
    if (config.isPowered) {
        executeWheelVelocity(0,0);
    }
}

bool MecanumWheel::allMotorsOnline() const {
    return config.isPowered? driveMotor.isMotorOnline() : false;
}

float MecanumWheel::getDriveVelocity() const {
    return config.isPowered? rpmToMps(driveMotor.getShaftRPM()) : 0.0f;
}

float MecanumWheel::getDriveRPM() const {
    return config.isPowered? driveMotor.getShaftRPM() : 0.0f;
}

int MecanumWheel::getNumMotors() const {
    return 1;
}
}  // namespace chassis
}  // namespace aruwsrc
