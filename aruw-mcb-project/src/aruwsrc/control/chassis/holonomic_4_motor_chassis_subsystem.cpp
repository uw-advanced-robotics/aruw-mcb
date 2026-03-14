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

#include "holonomic_4_motor_chassis_subsystem.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/communication/serial/remote.hpp"

#include "aruwsrc/communication/sensors/current/acs712_current_sensor_config.hpp"

#include "holonomic_chassis_subsystem.hpp"

using namespace tap::algorithms;

namespace aruwsrc::control::chassis
{
Holonomic4MotorChassisSubsystem::Holonomic4MotorChassisSubsystem(
    tap::Drivers* drivers,
    tap::communication::sensors::current::CurrentSensorInterface* currentSensor,
    tap::communication::sensors::voltage::VoltageSensorInterface* voltageSensor,
    Motor& leftFrontMotor,
    Motor& leftBackMotor,
    Motor& rightFrontMotor,
    Motor& rightBackMotor,
    tap::algorithms::SmoothPidConfig wheelVelocityPidConfig,
    communication::can::cap_bank::CapacitorBank* capacitorBank)
    : HolonomicChassisSubsystem(drivers, currentSensor, voltageSensor, capacitorBank),
      velocityPid{
          tap::algorithms::SmoothPid(wheelVelocityPidConfig),
          tap::algorithms::SmoothPid(wheelVelocityPidConfig),
          tap::algorithms::SmoothPid(wheelVelocityPidConfig),
          tap::algorithms::SmoothPid(wheelVelocityPidConfig)},
      velocityPidErrors{0, 0, 0, 0},
      leftFrontMotor(leftFrontMotor),
      leftBackMotor(leftBackMotor),
      rightFrontMotor(rightFrontMotor),
      rightBackMotor(rightBackMotor)
{
    motors[LF] = &leftFrontMotor;
    motors[RF] = &rightFrontMotor;
    motors[LB] = &leftBackMotor;
    motors[RB] = &rightBackMotor;
}

void Holonomic4MotorChassisSubsystem::initialize()
{
    for (int i = 0; i < getNumChassisMotors(); i++)
    {
        motors[i]->initialize();
    }
}

void Holonomic4MotorChassisSubsystem::setDesiredOutput(float x, float y, float r)
{
    calculateOutput(
        x,
        y,
        r,
        getMaxWheelSpeed(
            drivers->refSerial.getRefSerialReceivingData(),
            HolonomicChassisSubsystem::getChassisPowerLimit(drivers)));
}
modm::Matrix<float, 3, 1> state;
void Holonomic4MotorChassisSubsystem::refresh()
{
    for (int i = 0; i < getNumChassisMotors(); i++)
    {
        updateMotorRpmPid(i);
    }

    state = getActualVelocityChassisRelative();

    limitChassisPower();
}

void Holonomic4MotorChassisSubsystem::limitChassisPower()
{
    int NUM_MOTORS = getNumChassisMotors();

    // use power limiting object to compute initial power limiting fraction
    currentSensor->update();
    float powerLimitFrac = chassisPowerLimiter.getPowerLimitRatio();

    // total velocity error for all wheels
    float totalError = 0.0f;
    for (int i = 0; i < NUM_MOTORS; i++)
    {
        totalError += abs(velocityPidErrors[i]);
    }

    bool totalErrorZero = compareFloatClose(0.0f, totalError, 1E-3);

    // compute modified power limiting fraction based on velocity PID error
    // motors with greater error should be allocated a larger fraction of the powerLimitFrac
    for (int i = 0; i < NUM_MOTORS; i++)
    {
        // Compared to the other wheels, fraction of how much velocity PID error there is for a
        // single motor. Some value between [0, 1]. The sum of all computed velocityErrorFrac
        // values for all motors is 1.
        float velocityErrorFrac =
            totalErrorZero ? (1.0f / NUM_MOTORS) : (abs(velocityPidErrors[i]) / totalError);
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

void Holonomic4MotorChassisSubsystem::calculateOutput(
    float x,
    float y,
    float r,
    float maxWheelSpeed)
{
    // this is the distance between the center of the chassis to the wheel
    float chassisRotationRatio = WHEELBASE_RADIUS;

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

void Holonomic4MotorChassisSubsystem::updateMotorRpmPid(int i)
{
    // We divide by the gearbox ratio here because the PID is currently tuned for the internal RPM
    // and not the wheel rpm
    velocityPidErrors[i] = *desiredWheelRPM[i] - motors[i]->getEncoder()->getVelocity() * 60.0f /
                                                     M_TWOPI / CHASSIS_GEARBOX_RATIO;

    // dt is also set to 1 here because the current constants were for modm pid didn't use it
    velocityPid[i].runControllerDerivateError(velocityPidErrors[i], 1);

    float value =
        VELOCITY_PID_KV * (*desiredWheelRPM[i]) + velocityPid[i].getOutput() + VELOCITY_PID_KS;
    motors[i]->setDesiredOutput(value);
}

modm::Matrix<float, 3, 1> Holonomic4MotorChassisSubsystem::getActualVelocityChassisRelative() const
{
    modm::Matrix<float, MODM_ARRAY_SIZE(motors), 1> wheelVelocity;

    wheelVelocity[LF][0] = leftFrontMotor.getEncoder()->getVelocity();
    wheelVelocity[RF][0] = rightFrontMotor.getEncoder()->getVelocity();
    wheelVelocity[LB][0] = leftBackMotor.getEncoder()->getVelocity();
    wheelVelocity[RB][0] = rightBackMotor.getEncoder()->getVelocity();
    return wheelVelToChassisVelMat * wheelVelocity;
}

modm::Matrix<float, 3, 1> Holonomic4MotorChassisSubsystem::getDesiredVelocityChassisRelative() const
{
    return wheelVelToChassisVelMat * convertRawRPM(desiredWheelRPM);
}

}  // namespace aruwsrc::control::chassis
