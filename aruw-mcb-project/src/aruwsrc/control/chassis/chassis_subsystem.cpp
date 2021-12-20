/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

/*
 * Copyright (c) 2019 Sanger_X
 */

#include "chassis_subsystem.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/communication/serial/remote.hpp"

#include "aruwsrc/drivers.hpp"

using namespace tap::algorithms;

namespace aruwsrc
{
namespace chassis
{
ChassisSubsystem::ChassisSubsystem(
    aruwsrc::Drivers* drivers,
    tap::motor::MotorId leftFrontMotorId,
    tap::motor::MotorId leftBackMotorId,
    tap::motor::MotorId rightFrontMotorId,
    tap::motor::MotorId rightBackMotorId,
    tap::gpio::Analog::Pin currentPin)
    : tap::control::chassis::ChassisSubsystemInterface(drivers),
      leftFrontVelocityPid(
          VELOCITY_PID_KP,
          VELOCITY_PID_KI,
          VELOCITY_PID_KD,
          VELOCITY_PID_MAX_ERROR_SUM,
          VELOCITY_PID_MAX_OUTPUT),
      leftBackVelocityPid(
          VELOCITY_PID_KP,
          VELOCITY_PID_KI,
          VELOCITY_PID_KD,
          VELOCITY_PID_MAX_ERROR_SUM,
          VELOCITY_PID_MAX_OUTPUT),
      rightFrontVelocityPid(
          VELOCITY_PID_KP,
          VELOCITY_PID_KI,
          VELOCITY_PID_KD,
          VELOCITY_PID_MAX_ERROR_SUM,
          VELOCITY_PID_MAX_OUTPUT),
      rightBackVelocityPid(
          VELOCITY_PID_KP,
          VELOCITY_PID_KI,
          VELOCITY_PID_KD,
          VELOCITY_PID_MAX_ERROR_SUM,
          VELOCITY_PID_MAX_OUTPUT),
      chassisRotationErrorKalman(1.0f, 1.0f),
      leftFrontMotor(drivers, leftFrontMotorId, CAN_BUS_MOTORS, false, "left front drive motor"),
      leftBackMotor(drivers, leftBackMotorId, CAN_BUS_MOTORS, false, "left back drive motor"),
      rightFrontMotor(drivers, rightFrontMotorId, CAN_BUS_MOTORS, false, "right front drive motor"),
      rightBackMotor(drivers, rightBackMotorId, CAN_BUS_MOTORS, false, "right back drive motor"),
      chassisPowerLimiter(
          drivers,
          currentPin,
          MAX_ENERGY_BUFFER,
          ENERGY_BUFFER_LIMIT_THRESHOLD,
          ENERGY_BUFFER_CRIT_THRESHOLD,
          POWER_CONSUMPTION_THRESHOLD,
          CURRENT_ALLOCATED_FOR_ENERGY_BUFFER_LIMITING,
          motorConstants)
{
    constexpr float A = (WIDTH_BETWEEN_WHEELS_X + WIDTH_BETWEEN_WHEELS_Y == 0)
                            ? 1
                            : 2 / (WIDTH_BETWEEN_WHEELS_X + WIDTH_BETWEEN_WHEELS_Y);
    wheelVelToChassisVelMat[0][0] = 1;
    wheelVelToChassisVelMat[0][1] = -1;
    wheelVelToChassisVelMat[0][2] = 1;
    wheelVelToChassisVelMat[0][3] = -1;
    wheelVelToChassisVelMat[1][0] = 1;
    wheelVelToChassisVelMat[1][1] = 1;
    wheelVelToChassisVelMat[1][2] = -1;
    wheelVelToChassisVelMat[1][3] = -1;
    wheelVelToChassisVelMat[2][0] = 1.0 / A;
    wheelVelToChassisVelMat[2][1] = 1.0 / A;
    wheelVelToChassisVelMat[2][2] = 1.0 / A;
    wheelVelToChassisVelMat[2][3] = 1.0 / A;
    wheelVelToChassisVelMat *= (WHEEL_RADIUS / 4);

    motors[LF] = &leftFrontMotor;
    motors[RF] = &rightFrontMotor;
    motors[LB] = &leftBackMotor;
    motors[RB] = &rightBackMotor;
}

void ChassisSubsystem::initialize()
{
    // All of these DjiMotors are registered on CAN_BUS2
    leftBackMotor.initialize();    // Motor3: 0x203 = 515
    leftFrontMotor.initialize();   // Motor2: 0x202 = 514
    rightBackMotor.initialize();   // Motor4: 0x204 = 516
    rightFrontMotor.initialize();  // Motor1: 0x201 = 513
}

void ChassisSubsystem::setDesiredOutput(float x, float y, float r)
{
    mecanumDriveCalculate(x, y, r, MAX_WHEEL_SPEED_SINGLE_MOTOR);
}

void ChassisSubsystem::refresh()
{
    updateMotorRpmPid(&leftFrontVelocityPid, &leftFrontMotor, *desiredWheelRPM[LF]);
    updateMotorRpmPid(&rightFrontVelocityPid, &rightFrontMotor, *desiredWheelRPM[RF]);
    updateMotorRpmPid(&leftBackVelocityPid, &leftBackMotor, *desiredWheelRPM[LB]);
    updateMotorRpmPid(&rightBackVelocityPid, &rightBackMotor, *desiredWheelRPM[RB]);
    chassisPowerLimiter.performPowerLimiting(motors, MODM_ARRAY_SIZE(motors));
}

void ChassisSubsystem::mecanumDriveCalculate(float x, float y, float r, float maxWheelSpeed)
{
    // this is the distance between the center of the chassis to the wheel
    float chassisRotationRatio = sqrtf(
        powf(WIDTH_BETWEEN_WHEELS_X / 2.0f, 2.0f) + powf(WIDTH_BETWEEN_WHEELS_Y / 2.0f, 2.0f));

    // to take into account the location of the turret so we rotate around the turret rather
    // than the center of the chassis, we calculate the offset and than multiply however
    // much we want to rotate by
    float leftFrontRotationRatio =
        modm::toRadian(chassisRotationRatio - GIMBAL_X_OFFSET - GIMBAL_Y_OFFSET);
    float rightFroneRotationRatio =
        modm::toRadian(chassisRotationRatio - GIMBAL_X_OFFSET + GIMBAL_Y_OFFSET);
    float leftBackRotationRatio =
        modm::toRadian(chassisRotationRatio + GIMBAL_X_OFFSET - GIMBAL_Y_OFFSET);
    float rightBackRotationRatio =
        modm::toRadian(chassisRotationRatio + GIMBAL_X_OFFSET + GIMBAL_Y_OFFSET);

    float chassisRotateTranslated = modm::toDegree(r) / chassisRotationRatio;
    desiredWheelRPM[LF][0] = limitVal<float>(
        y + x + chassisRotateTranslated * leftFrontRotationRatio,
        -maxWheelSpeed,
        maxWheelSpeed);
    desiredWheelRPM[RF][0] = limitVal<float>(
        y - x + chassisRotateTranslated * rightFroneRotationRatio,
        -maxWheelSpeed,
        maxWheelSpeed);
    desiredWheelRPM[LB][0] = limitVal<float>(
        -y + x + chassisRotateTranslated * leftBackRotationRatio,
        -maxWheelSpeed,
        maxWheelSpeed);
    desiredWheelRPM[RB][0] = limitVal<float>(
        -y - x + chassisRotateTranslated * rightBackRotationRatio,
        -maxWheelSpeed,
        maxWheelSpeed);

    desiredRotation = r;
}

void ChassisSubsystem::updateMotorRpmPid(
    modm::Pid<float>* pid,
    tap::motor::DjiMotor* const motor,
    float desiredRpm)
{
    pid->update(desiredRpm - motor->getShaftRPM());
    motor->setDesiredOutput(pid->getValue());
}

float ChassisSubsystem::chassisSpeedRotationPID(float currentAngleError)
{
    float currentFilteredAngleErrorPrevious = chassisRotationErrorKalman.getLastFiltered();
    float currentFilteredAngleError = chassisRotationErrorKalman.filterData(currentAngleError);

    // P
    float currRotationPidP = currentAngleError * AUTOROTATION_PID_KP;
    currRotationPidP = limitVal(currRotationPidP, -AUTOROTATION_PID_MAX_P, AUTOROTATION_PID_MAX_P);

    // D
    float currentRotationPidD =
        (currentFilteredAngleError - currentFilteredAngleErrorPrevious) * AUTOROTATION_PID_KD;

    currentRotationPidD =
        limitVal(currentRotationPidD, -AUTOROTATION_PID_MAX_D, AUTOROTATION_PID_MAX_D);

    float wheelRotationSpeed = limitVal(
        currRotationPidP + currentRotationPidD,
        -AUTOROTATION_PID_MAX_OUTPUT,
        AUTOROTATION_PID_MAX_OUTPUT);

    return wheelRotationSpeed;
}

float ChassisSubsystem::calculateRotationTranslationalGain(float chassisRotationDesiredWheelspeed)
{
    // what we will multiply x and y speed by to take into account rotation
    float rTranslationalGain = 1.0f;

    // the x and y movement will be slowed by a fraction of auto rotation amount for maximizing
    // power consumption when the wheel rotation speed for chassis rotationis greater than the
    // MIN_ROTATION_THRESHOLD
    if (fabsf(chassisRotationDesiredWheelspeed) > MIN_ROTATION_THRESHOLD)
    {
        // power(max revolve speed - specified revolve speed, 2)
        // / power(max revolve speed, 2)
        rTranslationalGain = powf(
            ChassisSubsystem::MAX_WHEEL_SPEED_SINGLE_MOTOR + MIN_ROTATION_THRESHOLD -
                fabsf(chassisRotationDesiredWheelspeed) /
                    ChassisSubsystem::MAX_WHEEL_SPEED_SINGLE_MOTOR,
            2.0f);
        rTranslationalGain = tap::algorithms::limitVal<float>(rTranslationalGain, 0.0f, 1.0f);
    }
    return rTranslationalGain;
}

modm::Matrix<float, 3, 1> ChassisSubsystem::getDesiredVelocityChassisRelative() const
{
    return wheelVelToChassisVelMat * convertRawRPM(desiredWheelRPM);
}

modm::Matrix<float, 3, 1> ChassisSubsystem::getActualVelocityChassisRelative() const
{
    modm::Matrix<float, 4, 1> wheelVelocity;
    wheelVelocity[0][0] = leftFrontMotor.getShaftRPM();
    wheelVelocity[1][0] = rightFrontMotor.getShaftRPM();
    wheelVelocity[2][0] = leftBackMotor.getShaftRPM();
    wheelVelocity[3][0] = rightBackMotor.getShaftRPM();
    return wheelVelToChassisVelMat * convertRawRPM(wheelVelocity);
}

void ChassisSubsystem::getVelocityWorldRelative(
    modm::Matrix<float, 3, 1>& chassisRelativeVelocity,
    float chassisHeading) const
{
    modm::Matrix<float, 3, 3> transform;
    float headingCos = cosf(chassisHeading);
    float headingSin = sinf(chassisHeading);
    headingCos = compareFloatClose(headingCos, 0.0f, 1e-6) ? 0.0f : headingCos;
    headingSin = compareFloatClose(headingSin, 0.0f, 1e-6) ? 0.0f : headingSin;

    transform[0][0] = headingCos;
    transform[1][0] = headingSin;
    transform[2][0] = 0;
    transform[0][1] = -headingSin;
    transform[1][1] = headingCos;
    transform[2][1] = 0;
    transform[0][2] = 0;
    transform[1][2] = 0;
    transform[2][2] = 1;
    chassisRelativeVelocity = transform * chassisRelativeVelocity;
}

void ChassisSubsystem::onHardwareTestStart() { setDesiredOutput(0, 0, 0); }

}  // namespace chassis

}  // namespace aruwsrc
