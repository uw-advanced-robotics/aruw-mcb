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

#include "balancing_chassis_subsystem.hpp"

#include "aruwsrc/communication/sensors/current/acs712_current_sensor_config.hpp"
#include "aruwsrc/control/turret/constants/turret_constants.hpp"

namespace aruwsrc::chassis
{
BalancingChassisSubsystem::BalancingChassisSubsystem(
    tap::Drivers* drivers,
    aruwsrc::can::TurretMCBCanComm& turretMCB,
    const aruwsrc::control::turret::TurretMotor& pitchMotor,
    const aruwsrc::control::turret::TurretMotor& yawMotor,
    aruwsrc::control::motion::FiveBarLinkage* fivebarLeft,
    aruwsrc::control::motion::FiveBarLinkage* fivebarRight,
    tap::motor::MotorInterface* driveWheelLeft,
    tap::motor::MotorInterface* driveWheelRight,
    tap::gpio::Analog::Pin currentPin)
    : tap::control::chassis::ChassisSubsystemInterface(drivers),
      rotationPid(AUTOROTATION_PID),
      turretMCB(turretMCB),
      pitchMotor(pitchMotor),
      yawMotor(yawMotor),
      currentSensor(
          {&drivers->analog,
           currentPin,
           aruwsrc::communication::sensors::current::ACS712_CURRENT_SENSOR_MV_PER_MA,
           aruwsrc::communication::sensors::current::ACS712_CURRENT_SENSOR_ZERO_MA,
           aruwsrc::communication::sensors::current::ACS712_CURRENT_SENSOR_LOW_PASS_ALPHA}),
      chassisPowerLimiter(
          drivers,
          &currentSensor,
          STARTING_ENERGY_BUFFER,
          ENERGY_BUFFER_LIMIT_THRESHOLD,
          ENERGY_BUFFER_CRIT_THRESHOLD),
      fivebarLeft(fivebarLeft),
      fivebarRight(fivebarRight),
      driveWheelLeft(driveWheelLeft),
      driveWheelRight(driveWheelRight)
{
}

void BalancingChassisSubsystem::initialize()
{
    velocityRamper = tap::algorithms::Ramp();
    currentX = 0;
    desiredV = velocityRamper.getValue();
    desiredR = 0;
    desiredZRamper = tap::algorithms::Ramp(fivebarLeft->getDefaultPosition().getY());
    desiredZ = desiredZRamper.getValue();
    fivebarLeft->initialize();
    driveWheelLeft->initialize();
    fivebarRight->initialize();
    driveWheelRight->initialize();
    balanceAttemptCooldown.restart(BALANCE_ATTEMPT_COOLDOWN_DURATION);
}

void BalancingChassisSubsystem::refresh()
{
    /* 1. Compute dt and Update Current State */
    uint32_t currentTime = tap::arch::clock::getTimeMilliseconds();
    int32_t dt = currentTime - prevTime;
    prevTime = currentTime;
    fivebarLeft->refresh();
    fivebarRight->refresh();
    computeState(dt);

    /* 2. Compute Setpoints */
    switch (balancingState)
    {
        case BALANCING:
            updateBalancing(dt);
            break;
        case FALLEN_MOVING:
            updateFallenMoving(dt);
            break;
        case FALLEN_NOT_MOVING:
            updateFallenNotMoving(dt);
            break;
        case STANDING_UP:
            updateStandingUp(dt);
            break;
        case JUMPING:
            updateJumping(dt);
            break;
        case HOMING:
            homeLegs(dt);
            break;
        case RECOVERY:
            updateRecovery(dt);
            break;
        case FALLING:
            updateFalling(dt);
            break;
    };

    // Run Outputs to motors
    int32_t driveWheelOutputRight = wheelTorqueRight / .3 * 16384 / 20;
    int32_t driveWheelOutputLeft =
        wheelTorqueLeft / .3 * 16384 / 20;  // convert from Current to output
    if (armed)
    {
        driveWheelLeft->setDesiredOutput(driveWheelOutputLeft);
        driveWheelRight->setDesiredOutput(driveWheelOutputRight);
    }
    else
    {
        setLegsRetracted(dt);
        stopChassis();
    };
    fivebarController();
    if (driveWheelLeft->isMotorOnline() && driveWheelRight->isMotorOnline())
    {
        // do this here for safety. Only called once per subsystem. Don't arm leg motors until wheel
        // motors are also online.
        static_cast<aruwsrc::motor::Tmotor_AK809*>(fivebarLeft->getMotor1())->sendCanMessage();
        static_cast<aruwsrc::motor::Tmotor_AK809*>(fivebarLeft->getMotor2())->sendCanMessage();
        static_cast<aruwsrc::motor::Tmotor_AK809*>(fivebarRight->getMotor1())->sendCanMessage();
        static_cast<aruwsrc::motor::Tmotor_AK809*>(fivebarRight->getMotor2())->sendCanMessage();
    }
}

void BalancingChassisSubsystem::setSafeBehavior()
{
    driveWheelLeft->setDesiredOutput(0);
    driveWheelRight->setDesiredOutput(0);
    disarmChassis();
}

void BalancingChassisSubsystem::getAngles(uint32_t dt)
{
    // Value from [0, 2Pi]
    float currentTurretPitch = pitchMotor.getChassisFrameMeasuredAngle().getValue() -
                               aruwsrc::control::turret::PITCH_MOTOR_CONFIG.startAngle -
                               aruwsrc::control::turret::CHASSIS_FALL_OVER_OFFSET;
    if (currentTurretPitch > M_PI) currentTurretPitch -= M_TWOPI;
    float currentTurretYaw = yawMotor.getChassisFrameMeasuredAngle().getValue();
    if (currentTurretYaw > M_PI) currentTurretYaw -= M_TWOPI;

    float worldRelativeTurretPitch = turretMCB.getPitch();
    float worldRelativeTurretRoll = turretMCB.getRoll();
    float worldRelativeTurretYaw = turretMCB.getYaw();

    tap::algorithms::transforms::Transform<World, Turret> worldToTurret =
        tap::algorithms::transforms::Transform<World, Turret>(
            0,
            0,
            0,
            worldRelativeTurretRoll,
            worldRelativeTurretPitch,
            worldRelativeTurretYaw);

    // Define the transformation from Turret to Chassis based on turret encoders
    chassisToTurret.updateRotation(0, currentTurretPitch, currentTurretYaw);
    tap::algorithms::transforms::Transform<World, Chassis> worldToChassis =
        tap::algorithms::transforms::compose(worldToTurret, chassisToTurret.getInverse());

    roll = worldToChassis.getRoll();
    pitch = worldToChassis.getPitch();
    yaw = worldToChassis.getYaw();

    float rollRateNew = (roll - rollPrev) * 1'000 / dt;
    rollPrev = roll;
    rollRate = lowPassFilter(rollRate, rollRateNew, .1);

    float pitchRateNew = (pitch - pitchPrev) * 1'000 / dt;
    pitchPrev = pitch;
    pitchRate = lowPassFilter(pitchRate, pitchRateNew, .1);

    // Unwrap the yaw. We can safely assume that yaw won't change by half a rotation in 2ms.
    if (yaw - yawPrev > M_PI) yawPrev += M_TWOPI;
    if (yawPrev - yaw > M_PI) yawPrev -= M_TWOPI;
    float yawRateNew = (yaw - yawPrev) * 1000.0f / dt;
    yawPrev = yaw;
    yawRate = lowPassFilter(yawRate, yawRateNew, .1);
}

void BalancingChassisSubsystem::computeState(uint32_t dt)
{
    getAngles(dt);

    velocityRamper.update(dt * MAX_ACCELERATION / 1000);
    desiredV = velocityRamper.getValue();
    desiredZRamper.update(dt * Z_RAMP_RATE / 1000);
    desiredZ = limitVal(
        desiredZRamper.getValue(),
        fivebarLeft->getDefaultPosition().getY(),
        CHASSIS_HEIGHTS.second);

    /**
     * Increment our desired X position based on the desired velocity, if our desired velocity
     * changes to 0, reset the desired position
     */
    if (desiredV == 0 && prevVdesired != 0)
    {
        desiredX = currentX;
    }
    else if (desiredV != 0)
    {
        desiredX += desiredV * dt / 1'000;
    }
    prevVdesired = desiredV;

    // World relative wheel position
    currentZLeft = fivebarLeft->getCurrentPosition().getX() * sin(pitch) +
                   fivebarLeft->getCurrentPosition().getY() * cos(pitch);
    currentZRight = fivebarRight->getCurrentPosition().getX() * sin(pitch) +
                    fivebarRight->getCurrentPosition().getY() * cos(pitch);
    float xLeft = fivebarLeft->getCurrentPosition().getX() * cos(pitch) +
                  fivebarLeft->getCurrentPosition().getY() * sin(pitch);
    float xRight = fivebarRight->getCurrentPosition().getX() * cos(pitch) +
                   fivebarRight->getCurrentPosition().getY() * sin(pitch);
    // link angle in world-frame
    tlLeft = lowPassFilter(tlLeft, atan2(xLeft, currentZLeft), .4);
    tlRight = lowPassFilter(tlRight, atan2(xRight, currentZRight), .4);
    tl = (tlLeft + tlRight) / 2;

    float tlRateNew = (tl - tlPrev) * 1000 / dt;
    tlRate = lowPassFilter(tlRate, tlRateNew, .6);
    tlPrev = tl;

    desiredV = velocityRamper.getValue();
    wheelPosLeft = driveWheelLeft->getPositionUnwrapped() * CHASSIS_GEARBOX_RATIO * WHEEL_RADIUS;
    wheelPosRight = driveWheelRight->getPositionUnwrapped() * CHASSIS_GEARBOX_RATIO * WHEEL_RADIUS;
    wheelVelLeft = lowPassFilter(wheelVelLeft, (wheelPosLeft - wheelPosPrevLeft) * 1000 / dt, .1);
    wheelVelRight =
        lowPassFilter(wheelVelRight, (wheelPosRight - wheelPosPrevRight) * 1000 / dt, .1);
    wheelPosPrevLeft = wheelPosLeft;
    wheelPosPrevRight = wheelPosRight;

    currentX = (wheelPosLeft + wheelPosRight) / 2;
    currentV = (wheelVelLeft + wheelVelRight) / 2;

    currentR = (wheelVelRight - wheelVelLeft) / 2 / WIDTH_BETWEEN_WHEELS_Y;

    // currentZ = abs(currentZRight) < abs(currentZLeft) ? currentZLeft : currentZRight;
    currentZ = (currentZLeft + currentZRight) / 2;
    airborneDetector();
}

void BalancingChassisSubsystem::runHardwareTests() {}

void BalancingChassisSubsystem::homeLegs(uint32_t dt)
{
    if (balancingState != HOMING) return;
    // 1. PID legs to an angle slightly more towards their home until they reach it.
    // Fronts want to rotate negative
    static aruwsrc::motor::Tmotor_AK809* legMotors[4] = {
        static_cast<aruwsrc::motor::Tmotor_AK809*>(fivebarLeft->getMotor1()),
        static_cast<aruwsrc::motor::Tmotor_AK809*>(fivebarLeft->getMotor2()),
        static_cast<aruwsrc::motor::Tmotor_AK809*>(fivebarRight->getMotor1()),
        static_cast<aruwsrc::motor::Tmotor_AK809*>(fivebarRight->getMotor2())};
    int i = 0;
    for (i = 0; i < 4; i++)
    {  // 2. When the PID output exceeds some current, stop. Set the home, and Reset the PID to hold
        // the current position.
        float motorAngleSetpoint = ((i % 2 == 0) ? -1 : 1) * modm::toRadian(5);
        float output = legHomingPid[i].runController(
            motorAngleSetpoint,
            legMotors[i]->getShaftRPM() * M_TWOPI / 60,
            dt);
        legMotors[i]->setDesiredOutput(output);
        if (compareFloatClose(legMotors[i]->getShaftRPM(), 0, 1e-3) &&
            !compareFloatClose(output, 0, STALL_CURRENT))
        {
            legStalled[i] = true;
            legMotors[i]->sendPositionHomeGetMessage();
        }
        else
        {
            legStalled[i] = false;
        }
    }

    if (legStalled[0] && legStalled[1] && legStalled[2] && legStalled[3])
    {
        legStalled[0] = false;
        legStalled[1] = false;
        legStalled[2] = false;
        legStalled[3] = false;
        balancingState = FALLEN_MOVING;
    }
}

void BalancingChassisSubsystem::setLegsRetracted(uint32_t dt)
{
    desiredLinkForceLeft = retractionPid[0].runControllerDerivateError(
        fivebarLeft->getDefaultPosition().getLength() - fivebarLeft->getCurrentLength(),
        dt);
    desiredLinkForceRight = retractionPid[1].runControllerDerivateError(
        fivebarRight->getDefaultPosition().getLength() - fivebarRight->getCurrentLength(),
        dt);
    desiredLinkTorqueLeft = retractionAnglePid[0].runControllerDerivateError(
        M_PI_2 - fivebarLeft->getCurrentAngle(),
        dt);
    desiredLinkTorqueRight = retractionAnglePid[1].runControllerDerivateError(
        M_PI_2 - fivebarRight->getCurrentAngle(),
        dt);
}

void BalancingChassisSubsystem::updateBalancing(uint32_t dt)
{
    // 1. Run LQR controller
    float stateData[6] = {
        tl,
        deadZone(tlRate, .1f),
        limitVal(currentX - desiredX, -2.0f, 2.0f),
        currentV - desiredV,
        -pitch,
        -pitchRate,
    };
    CMSISMat<6, 1> stateVector(stateData);
    CMSISMat<2, 6> LQR_K = LQR_K_SLOPE * currentZ + LQR_K_YINT;
    CMSISMat<2, 1> torques = -LQR_K * stateVector;
    wheelTorqueLeft = wheelTorqueRight = limitVal(torques.data[0] / 2, -2.0f, 2.0f);
    desiredLinkTorqueLeft = desiredLinkTorqueRight = limitVal(-torques.data[1] / 2, -10.0f, 10.0f);

    // 2. run PID Superimposers
    float yawAdjustment = yawPid.runController(desiredR, yawRate, dt);
    float linkAngleAdjustment =
        linkAngleMismatchPid.runControllerDerivateError(tlRight - tlLeft, dt);
    float legForce = heightPid.runControllerDerivateError(desiredZ - currentZ, dt);
    float rollAdjustment = rollPid.runController(-roll, rollRate, dt);
    // 3. Superimpose
    float gravityFeedForward = ACCELERATION_GRAVITY * MASS_CHASSIS / 2 * cos(tl);

    // gravityFeedForward = 0;

    desiredLinkTorqueRight += linkAngleAdjustment;
    desiredLinkTorqueLeft -= linkAngleAdjustment;

    desiredLinkForceLeft = legForce + rollAdjustment + gravityFeedForward;
    desiredLinkForceRight = legForce - rollAdjustment + gravityFeedForward;

    wheelTorqueRight += yawAdjustment;
    wheelTorqueLeft -= yawAdjustment;

    // 4. transition states
    if (abs(pitch) > abs(FALLEN_ANGLE_THRESHOLD))
    {
        balancingState = FALLEN_MOVING;
    }
    // if (currentLinkForceLeft < FALLING_FORCE_THRESHOLD ||
    //     currentLinkForceRight < FALLING_FORCE_THRESHOLD)
    // {
    //     balancingState = FALLING;
    // }
}

void BalancingChassisSubsystem::updateFallenMoving(uint32_t dt)
{
    setLegsRetracted(dt);
    wheelTorqueLeft = wheelTorqueRight = 0;
    if (armed && abs(pitch) < abs(FALLEN_ANGLE_RETURN) &&
        abs(pitchRate) < abs(FALLEN_ANGLE_RATE_THRESHOLD))
    {
        balancingState = BALANCING;
    }
    if (compareFloatClose(currentV, 0, .1))
    {
        balancingState = FALLEN_NOT_MOVING;
    }
}
void BalancingChassisSubsystem::updateFallenNotMoving(uint32_t dt)
{
    setLegsRetracted(dt);
    wheelTorqueLeft = wheelTorqueRight = 0;
    if (!compareFloatClose(currentV, 0, .1))
    {
        balancingState = FALLEN_MOVING;
        return;
    }
    if (armed && abs(pitch) < abs(FALLEN_ANGLE_RETURN) &&
        abs(pitchRate) < abs(FALLEN_ANGLE_RATE_THRESHOLD))
    {
        // reset the x setpoint to avoid funniness after standing up
        desiredX = currentX;
        desiredZRamper.setTarget(CHASSIS_HEIGHTS.getFirst());
        desiredZRamper.setValue(CHASSIS_HEIGHTS.getFirst());
        balancingState = BALANCING;
    }
    else if (standupEnable && armed && balanceAttemptCooldown.isExpired())
    {
        balanceAttemptTimeout.restart(BALANCE_ATTEMPT_TIMEOUT_DURATION);
        balanceAttemptCooldown.restart(BALANCE_ATTEMPT_COOLDOWN_DURATION);
        balancingState = STANDING_UP;
    }
}
void BalancingChassisSubsystem::updateStandingUp(uint32_t dt)
{
    if (balanceAttemptTimeout.isExpired() || !armed)
    {
        balancingState = FALLEN_MOVING;
    }
    setLegsRetracted(dt);
    // Use nonlinear feedforward torque application
    float standupTorque = sin(pitch) * .347 * MASS_CHASSIS / 2 * 9.81 * STANDUP_TORQUE_GAIN;
    standupTorque = limitVal(standupTorque, -3.0f, 3.0f);
    wheelTorqueLeft = wheelTorqueRight = standupTorque;

    if (abs(pitch) < abs(FALLEN_ANGLE_RETURN) && abs(pitchRate) < abs(FALLEN_ANGLE_RATE_THRESHOLD))
    {
        balancingState = BALANCING;
        // reset the x setpoint to avoid funniness after standing up
        desiredX = currentX;
        desiredZRamper.setTarget(CHASSIS_HEIGHTS.getFirst());
        desiredZRamper.setValue(CHASSIS_HEIGHTS.getFirst());
    }
}
void BalancingChassisSubsystem::updateJumping(uint32_t dt)
{
    desiredZ = CHASSIS_HEIGHTS.getSecond();
    // This is a little sussy, but we want to keep balancing while we jump.
    updateBalancing(dt);
    // override the force PID and yeet
    heightPid.reset();
    desiredLinkForceLeft = MASS_CHASSIS / 2 * ACCELERATION_GRAVITY * JUMP_GRAV_GAIN;
    desiredLinkForceRight = MASS_CHASSIS / 2 * ACCELERATION_GRAVITY * JUMP_GRAV_GAIN;
    if (jumpTimeout.isExpired())
    {
        balancingState = FALLING;
        heightPid.reset();
    }
    else
    {
        balancingState = JUMPING;
    }
}
void BalancingChassisSubsystem::updateFalling(uint32_t dt)
{
    wheelTorqueRight = wheelTorqueLeft = 0;

    desiredZRamper.setTarget((CHASSIS_HEIGHTS.getFirst() + CHASSIS_HEIGHTS.getSecond()) / 2);
    float legForce = heightPid.runControllerDerivateError(desiredZ - currentZ, dt);
    desiredLinkForceLeft = desiredLinkForceRight = legForce;

    desiredLinkTorqueLeft = retractionAnglePid[0].runControllerDerivateError(0 - tlLeft, dt);
    desiredLinkTorqueRight = retractionAnglePid[1].runControllerDerivateError(0 - tlRight, dt);

    if (compareFloatClose(legForce, 0, 20.0f))
    {
        desiredZRamper.setTarget(CHASSIS_HEIGHTS.getFirst());
        balancingState = BALANCING;
    }
}

void BalancingChassisSubsystem::updateRecovery(uint32_t dt) {}

void BalancingChassisSubsystem::fivebarController()
{
    float leftFData[2] = {desiredLinkForceLeft, desiredLinkTorqueLeft};
    float rightFData[2] = {desiredLinkForceRight, desiredLinkTorqueRight};

    CMSISMat<2, 1> leftF(leftFData);
    CMSISMat<2, 1> rightF(rightFData);

    CMSISMat<2, 1> leftMotorTorque = VMC_JACOBIAN * leftF;
    CMSISMat<2, 1> rightMotorTorque = VMC_JACOBIAN * rightF;

    fivebarLeft->getMotor1()->setDesiredOutput(1000 * leftMotorTorque.data[0]);
    fivebarLeft->getMotor2()->setDesiredOutput(1000 * leftMotorTorque.data[1]);
    fivebarRight->getMotor1()->setDesiredOutput(1000 * rightMotorTorque.data[0]);
    fivebarRight->getMotor2()->setDesiredOutput(1000 * rightMotorTorque.data[1]);
}

void BalancingChassisSubsystem::airborneDetector()
{
    float dataLeft[2] = {
        fivebarLeft->getMotor1()->getTorque() / 100 * aruwsrc::motor::AK809_TORQUE_CONSTANT,
        fivebarLeft->getMotor1()->getTorque() / 100 * aruwsrc::motor::AK809_TORQUE_CONSTANT};
    CMSISMat<2, 1> leftMotorTorques(dataLeft);
    CMSISMat<2, 1> leftLinkForces = VMC_JACOBIAN_INV * leftMotorTorques;

    float dataRight[2] = {
        fivebarRight->getMotor1()->getTorque() / 100 * aruwsrc::motor::AK809_TORQUE_CONSTANT,
        fivebarRight->getMotor1()->getTorque() / 100 * aruwsrc::motor::AK809_TORQUE_CONSTANT};
    CMSISMat<2, 1> rightMotorTorques(dataRight);

    CMSISMat<2, 1> rightLinkForces = VMC_JACOBIAN_INV * rightMotorTorques;
    currentLinkForceLeft = leftLinkForces.data[0];
    currentLinkForceRight = rightLinkForces.data[0];
}
}  // namespace aruwsrc::chassis
