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

#include "aruwlib/algorithms/math_user_utils.hpp"
#include "aruwlib/communication/remote.hpp"
#include "aruwlib/drivers.hpp"

using namespace aruwlib;
using namespace aruwlib::algorithms;

namespace aruwsrc
{
namespace chassis
{
ChassisSubsystem::ChassisSubsystem(
    aruwlib::Drivers* drivers,
    const ChassisMechanicalConstants& mechanicalConstants,
    const aruwlib::algorithms::PidConfigStruct& velocityPidConfig,
    const aruwlib::control::chassis::PowerLimiterConfig& powerLimiterConfig,
    float maxWheelSpeedSingleMotor,
    float chassisRevolvePidMaxP,
    float chassisRevolvePidMaxD,
    float chassisRevolvePidKD,
    float chassisRevolvePidMaxOutput,
    float minErrorRotationD,
    float minRotationThreshold,
    aruwlib::can::CanBus canBus,
    aruwlib::motor::MotorId leftFrontMotorId,
    aruwlib::motor::MotorId leftBackMotorId,
    aruwlib::motor::MotorId rightFrontMotorId,
    aruwlib::motor::MotorId rightBackMotorId,
    aruwlib::gpio::Analog::Pin currentPin)
    : aruwlib::control::chassis::ChassisSubsystemInterface(drivers),
      MAX_WHEEL_SPEED_SINGLE_MOTOR(maxWheelSpeedSingleMotor),
      MOTOR_GEARBOX_RATIO(mechanicalConstants.motorGearboxRatio),
      WIDTH_BETWEEN_WHEELS_X(mechanicalConstants.widthBetweenWheelsX),
      WIDTH_BETWEEN_WHEELS_Y(mechanicalConstants.widthBetweenWheelsY),
      WHEEL_RADIUS(mechanicalConstants.wheelRadius),
      GIMBAL_X_OFFSET(mechanicalConstants.gimbalXOffset),
      GIMBAL_Y_OFFSET(mechanicalConstants.gimbalYOffset),
      CHASSIS_REVOLVE_PID_MAX_P(chassisRevolvePidMaxP),
      CHASSIS_REVOLVE_PID_MAX_D(chassisRevolvePidMaxD),
      CHASSIS_REVOLVE_PID_KD(chassisRevolvePidKD),
      CHASSIS_REVOLVE_PID_MAX_OUTPUT(chassisRevolvePidMaxOutput),
      MIN_ERROR_ROTATION_D(minErrorRotationD),
      MIN_ROTATION_THRESHOLD(minRotationThreshold),
      leftFrontVelocityPid(
          velocityPidConfig.kp,
          velocityPidConfig.ki,
          velocityPidConfig.kd,
          velocityPidConfig.maxICumulative,
          velocityPidConfig.maxOutput),
      leftBackVelocityPid(
          velocityPidConfig.kp,
          velocityPidConfig.ki,
          velocityPidConfig.kd,
          velocityPidConfig.maxICumulative,
          velocityPidConfig.maxOutput),
      rightFrontVelocityPid(
          velocityPidConfig.kp,
          velocityPidConfig.ki,
          velocityPidConfig.kd,
          velocityPidConfig.maxICumulative,
          velocityPidConfig.maxOutput),
      rightBackVelocityPid(
          velocityPidConfig.kp,
          velocityPidConfig.ki,
          velocityPidConfig.kd,
          velocityPidConfig.maxICumulative,
          velocityPidConfig.maxOutput),
      chassisRotationErrorKalman(1.0f, 0.0f),
      leftFrontMotor(drivers, leftFrontMotorId, canBus, false, "left front drive motor"),
      leftBackMotor(drivers, leftBackMotorId, canBus, false, "left back drive motor"),
      rightFrontMotor(drivers, rightFrontMotorId, canBus, false, "right front drive motor"),
      rightBackMotor(drivers, rightBackMotorId, canBus, false, "right back drive motor"),
      chassisPowerLimiter(drivers, currentPin, powerLimiterConfig, motorConstants)
{
    const float A = (WIDTH_BETWEEN_WHEELS_X + WIDTH_BETWEEN_WHEELS_Y == 0)
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
        degreesToRadians(chassisRotationRatio - GIMBAL_X_OFFSET - GIMBAL_Y_OFFSET);
    float rightFroneRotationRatio =
        degreesToRadians(chassisRotationRatio - GIMBAL_X_OFFSET + GIMBAL_Y_OFFSET);
    float leftBackRotationRatio =
        degreesToRadians(chassisRotationRatio + GIMBAL_X_OFFSET - GIMBAL_Y_OFFSET);
    float rightBackRotationRatio =
        degreesToRadians(chassisRotationRatio + GIMBAL_X_OFFSET + GIMBAL_Y_OFFSET);

    float chassisRotateTranslated = radiansToDegrees(r) / chassisRotationRatio;
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
    aruwlib::motor::DjiMotor* const motor,
    float desiredRpm)
{
    pid->update(desiredRpm - motor->getShaftRPM());
    motor->setDesiredOutput(pid->getValue());
}

float ChassisSubsystem::chassisSpeedRotationPID(float currentAngleError, float kp)
{
    float currentFilteredAngleErrorPrevious = chassisRotationErrorKalman.getLastFiltered();
    float currentFilteredAngleError = chassisRotationErrorKalman.filterData(currentAngleError);

    // P
    float currRotationPidP = currentAngleError * kp;
    currRotationPidP = aruwlib::algorithms::limitVal<float>(
        currRotationPidP,
        -CHASSIS_REVOLVE_PID_MAX_P,
        CHASSIS_REVOLVE_PID_MAX_P);

    // D
    float currentRotationPidD = 0.0f;
    if (abs(currentFilteredAngleError) > MIN_ERROR_ROTATION_D)
    {
        float currentErrorRotation = currentFilteredAngleError - currentFilteredAngleErrorPrevious;

        currentRotationPidD = -(currentErrorRotation)*CHASSIS_REVOLVE_PID_KD;

        currentRotationPidD = aruwlib::algorithms::limitVal<float>(
            currentRotationPidD,
            -CHASSIS_REVOLVE_PID_MAX_D,
            CHASSIS_REVOLVE_PID_MAX_D);
    }

    float wheelRotationSpeed = aruwlib::algorithms::limitVal<float>(
        currRotationPidP + currentRotationPidD,
        -CHASSIS_REVOLVE_PID_MAX_OUTPUT,
        CHASSIS_REVOLVE_PID_MAX_OUTPUT);

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
            MAX_WHEEL_SPEED_SINGLE_MOTOR + MIN_ROTATION_THRESHOLD -
                fabsf(chassisRotationDesiredWheelspeed) / MAX_WHEEL_SPEED_SINGLE_MOTOR,
            2.0f);
        rTranslationalGain = aruwlib::algorithms::limitVal<float>(rTranslationalGain, 0.0f, 1.0f);
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
