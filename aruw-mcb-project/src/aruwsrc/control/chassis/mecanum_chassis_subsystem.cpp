/*
 * Copyright (c) 2020-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "mecanum_chassis_subsystem.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/communication/serial/remote.hpp"

#include "aruwsrc/communication/sensors/current/acs712_current_sensor_config.hpp"
#include "aruwsrc/drivers.hpp"

#include "holonomic_chassis_subsystem.hpp"

using namespace tap::algorithms;

namespace aruwsrc
{
namespace chassis
{

MecanumChassisSubsystem::MecanumChassisSubsystem(
    aruwsrc::Drivers* drivers,
    tap::motor::MotorId leftFrontMotorId,
    tap::motor::MotorId leftBackMotorId,
    tap::motor::MotorId rightFrontMotorId,
    tap::motor::MotorId rightBackMotorId,
    tap::gpio::Analog::Pin currentPin)
    : HolonomicChassisSubsystem(drivers, currentPin),
      velocityPid{
          modm::Pid<float>(
              VELOCITY_PID_KP,
              VELOCITY_PID_KI,
              VELOCITY_PID_KD,
              VELOCITY_PID_MAX_ERROR_SUM,
              VELOCITY_PID_MAX_OUTPUT),
          modm::Pid<float>(
              VELOCITY_PID_KP,
              VELOCITY_PID_KI,
              VELOCITY_PID_KD,
              VELOCITY_PID_MAX_ERROR_SUM,
              VELOCITY_PID_MAX_OUTPUT),
          modm::Pid<float>(
              VELOCITY_PID_KP,
              VELOCITY_PID_KI,
              VELOCITY_PID_KD,
              VELOCITY_PID_MAX_ERROR_SUM,
              VELOCITY_PID_MAX_OUTPUT),
          modm::Pid<float>(
              VELOCITY_PID_KP,
              VELOCITY_PID_KI,
              VELOCITY_PID_KD,
              VELOCITY_PID_MAX_ERROR_SUM,
              VELOCITY_PID_MAX_OUTPUT)},
      leftFrontMotor(drivers, leftFrontMotorId, CAN_BUS_MOTORS, false, "left front drive motor"),
      leftBackMotor(drivers, leftBackMotorId, CAN_BUS_MOTORS, false, "left back drive motor"),
      rightFrontMotor(drivers, rightFrontMotorId, CAN_BUS_MOTORS, false, "right front drive motor"),
      rightBackMotor(drivers, rightBackMotorId, CAN_BUS_MOTORS, false, "right back drive motor")
{
    wheelVelToChassisVelMat[X][LF] = 1;
    wheelVelToChassisVelMat[X][RF] = -1;
    wheelVelToChassisVelMat[X][LB] = 1;
    wheelVelToChassisVelMat[X][RB] = -1;
    wheelVelToChassisVelMat[Y][LF] = -1;
    wheelVelToChassisVelMat[Y][RF] = -1;
    wheelVelToChassisVelMat[Y][LB] = 1;
    wheelVelToChassisVelMat[Y][RB] = 1;
    wheelVelToChassisVelMat[R][LF] = -1.0 / WHEELBASE_HYPOTENUSE;
    wheelVelToChassisVelMat[R][RF] = -1.0 / WHEELBASE_HYPOTENUSE;
    wheelVelToChassisVelMat[R][LB] = -1.0 / WHEELBASE_HYPOTENUSE;
    wheelVelToChassisVelMat[R][RB] = -1.0 / WHEELBASE_HYPOTENUSE;
    wheelVelToChassisVelMat *= (WHEEL_RADIUS / 4);
    motors[LF] = &leftFrontMotor;
    motors[RF] = &rightFrontMotor;
    motors[LB] = &leftBackMotor;
    motors[RB] = &rightBackMotor;
}

void MecanumChassisSubsystem::initialize()
{
    for (size_t i = 0; i < MODM_ARRAY_SIZE(motors); i++)
    {
        motors[i]->initialize();
    }
}

void MecanumChassisSubsystem::setDesiredOutput(float x, float y, float r)
{
    mecanumDriveCalculate(
        x,
        y,
        r,
        HolonomicChassisSubsystem::getMaxWheelSpeed(
            drivers->refSerial.getRefSerialReceivingData(),
            drivers->refSerial.getRobotData().chassis.powerConsumptionLimit));
}

void MecanumChassisSubsystem::refresh()
{
    for (size_t i = 0; i < MODM_ARRAY_SIZE(motors); i++)
    {
        updateMotorRpmPid(&velocityPid[i], motors[i], *desiredWheelRPM[i]);
    }

    limitChassisPower();
}

void MecanumChassisSubsystem::limitChassisPower()
{
    static constexpr size_t NUM_MOTORS = MODM_ARRAY_SIZE(motors);

    // use power limiting object to compute initial power limiting fraction
    currentSensor.update();
    float powerLimitFrac = chassisPowerLimiter.getPowerLimitRatio();

    // short circuit if power limiting doesn't need to be applied
    if (compareFloatClose(1.0f, powerLimitFrac, 1E-3))
    {
        return;
    }

    // total velocity error for all wheels
    float totalError = 0.0f;
    for (size_t i = 0; i < NUM_MOTORS; i++)
    {
        totalError += abs(velocityPid[i].getLastError());
    }

    bool totalErrorZero = compareFloatClose(0.0f, totalError, 1E-3);

    // compute modified power limiting fraction based on velocity PID error
    // motors with greater error should be allocated a larger fraction of the powerLimitFrac
    for (size_t i = 0; i < NUM_MOTORS; i++)
    {
        // Compared to the other wheels, fraction of how much velocity PID error there is for a
        // single motor. Some value between [0, 1]. The sume of all computed velocityErrorFrac
        // values for all motors is 1.
        float velocityErrorFrac = totalErrorZero
                                      ? (1.0f / NUM_MOTORS)
                                      : (abs(velocityPid[i].getLastError()) / totalError);
        // Instead of just multiplying the desired output by powerLimitFrac, scale powerLimitFrac
        // based on the current velocity error. In this way, if the velocity error is large, the
        // motor requires more current to be directed to it than other motors. Without this
        // compensation, a total of NUM_MOTORS * powerLimitFrac fractional limiting is divided
        // evenly among NUM_MOTORS motors. Instead, divide this limiting based on the
        // velocityErrorFrac for each motor.
        float modifiedPowerLimitFrac =
            limitVal(NUM_MOTORS * powerLimitFrac * velocityErrorFrac, 0.0f, 1.0f);
        motors[i]->setDesiredOutput(motors[i]->getOutputDesired() * modifiedPowerLimitFrac);
    }
}

void MecanumChassisSubsystem::mecanumDriveCalculate(float x, float y, float r, float maxWheelSpeed)
{
    // this is the distance between the center of the chassis to the wheel
    float chassisRotationRatio = sqrtf(
        powf(WIDTH_BETWEEN_WHEELS_X / 2.0f, 2.0f) + powf(WIDTH_BETWEEN_WHEELS_Y / 2.0f, 2.0f));

    // to take into account the location of the turret so we rotate around the turret rather
    // than the center of the chassis, we calculate the offset and than multiply however
    // much we want to rotate by
    float leftFrontRotationRatio =
        modm::toRadian(chassisRotationRatio - GIMBAL_X_OFFSET - GIMBAL_Y_OFFSET);
    float rightFrontRotationRatio =
        modm::toRadian(chassisRotationRatio - GIMBAL_X_OFFSET + GIMBAL_Y_OFFSET);
    float leftBackRotationRatio =
        modm::toRadian(chassisRotationRatio + GIMBAL_X_OFFSET - GIMBAL_Y_OFFSET);
    float rightBackRotationRatio =
        modm::toRadian(chassisRotationRatio + GIMBAL_X_OFFSET + GIMBAL_Y_OFFSET);

    float chassisRotateTranslated = modm::toDegree(r) / chassisRotationRatio;
    desiredWheelRPM[LF][0] = limitVal(
        -y + x - chassisRotateTranslated * leftFrontRotationRatio,
        -maxWheelSpeed,
        maxWheelSpeed);
    desiredWheelRPM[RF][0] = limitVal(
        -y - x - chassisRotateTranslated * rightFrontRotationRatio,
        -maxWheelSpeed,
        maxWheelSpeed);
    desiredWheelRPM[LB][0] = limitVal(
        y + x - chassisRotateTranslated * leftBackRotationRatio,
        -maxWheelSpeed,
        maxWheelSpeed);
    desiredWheelRPM[RB][0] = limitVal(
        y - x - chassisRotateTranslated * rightBackRotationRatio,
        -maxWheelSpeed,
        maxWheelSpeed);

    desiredRotation = r;
}

void MecanumChassisSubsystem::updateMotorRpmPid(
    modm::Pid<float>* pid,
    tap::motor::DjiMotor* const motor,
    float desiredRpm)
{
    pid->update(desiredRpm - motor->getShaftRPM());
    motor->setDesiredOutput(pid->getValue());
}

modm::Matrix<float, 3, 1> MecanumChassisSubsystem::getActualVelocityChassisRelative() const
{
    modm::Matrix<float, MODM_ARRAY_SIZE(motors), 1> wheelVelocity;

    wheelVelocity[LF][0] = leftFrontMotor.getShaftRPM();
    wheelVelocity[RF][0] = rightFrontMotor.getShaftRPM();
    wheelVelocity[LB][0] = leftBackMotor.getShaftRPM();
    wheelVelocity[RB][0] = rightBackMotor.getShaftRPM();
    return wheelVelToChassisVelMat * convertRawRPM(wheelVelocity);
}

}  // namespace chassis

}  // namespace aruwsrc
