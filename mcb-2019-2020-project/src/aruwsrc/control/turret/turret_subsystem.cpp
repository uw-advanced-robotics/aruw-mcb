#include <algorithm>
#include <random>
#include "turret_subsystem.hpp"
#include "src/aruwlib/algorithms/math_user_utils.hpp"

#define DEGREE_TO_ENCODER(degree) ((8192 * degree) / 360)
#define ENCODER_TO_DEGREE(encoder) ((static_cast<float>(encoder) * 360.0f) / 8192.0f)

namespace aruwsrc
{

namespace control
{
    TurretSubsystem::TurretSubsystem() : 
        pitchMotor(PITCH_MOTOR_ID, CAN_BUS_MOTORS, true),
        yawMotor(YAW_MOTOR_ID, CAN_BUS_MOTORS, false),
        turretStatus(IDLE),
        pitchMotorPid(
            TURRET_PITCH_PID_P,
            TURRET_PITCH_PID_I,
            TURRET_PITCH_PID_D, 0.0f,
            TURRET_MAX_VOLTAGE_OUT
        ),
        yawMotorPid(
            TURRET_YAW_PID_P,
            TURRET_YAW_PID_I,
            TURRET_YAW_PID_D, 0.0f,
            TURRET_MAX_VOLTAGE_OUT
        ),
        currYawAngle(0.0f, 0.0f, 360.0f),
        currPitchAngle(0.0f, 0.0f, 360.0f),
        desiredYawAngle(TURRET_START_ANGLE, 0.0f, 360.0f),
        desiredPitchAngle(TURRET_START_ANGLE, 0.0f, 360.0f) {
        setDefaultCommand(modm::SmartPointer(turretManual));
    }

    void TurretSubsystem::pitchMotorToDegree(float degrees) {
        turretCV->pitchToEncoder(convertToUnwrappedEncoder(&pitchMotor, degrees));
    }

    void TurretSubsystem::yawMotorToDegree(float degrees) {
        turretCV->yawToEncoder(convertToUnwrappedEncoder(&yawMotor, degrees));
    }

    void TurretSubsystem::incPitchMotorByDegree(float degrees) {
        turretCV->pitchIncrementEncoder(DEGREE_TO_ENCODER(degrees));
    }

    void TurretSubsystem::incYawMotorByDegree(float degrees) {
        turretCV->yawIncrementEncoder(DEGREE_TO_ENCODER(degrees));
    }

    // TODO : units?
    void TurretSubsystem::setPitchVelocity(float velocity) {
        turretManual->pitchToVelocity(velocity);
    }

    // TODO : units?
    void TurretSubsystem::setYawVelocity(float velocity) {
        turretManual->yawToVelocity(velocity);
    }

    float TurretSubsystem::getYawAngle(void) {
        return getAngle(&yawMotor);
    }

    float TurretSubsystem::getPitchAngle(void) {
        return getAngle(&pitchMotor);
    }

    float TurretSubsystem::getYawVelocity(void) {
        return getVelocity(&yawMotor);
    }

    float TurretSubsystem::getPitchVelocity(void) {
        return getVelocity(&pitchMotor);
    }

    void TurretSubsystem::refresh() {
        updateTurretVals();
    }

    float TurretSubsystem::convertToUnwrappedEncoder(aruwlib::motor::DjiMotor *motor, float degrees) {
        float targetDegree = degrees < 0 ? 360 - fmod(degrees, 360) : fmod(degrees, 360);
        float relativePosition = fmod(DEGREE_TO_ENCODER(targetDegree) - motor->encStore.getEncoderWrapped(), 8192);
        return motor->encStore.getEncoderUnwrapped() + DEGREE_TO_ENCODER(relativePosition);
    }

    float TurretSubsystem::getAngle(aruwlib::motor::DjiMotor *motor) {
        return DEGREE_TO_ENCODER(motor->encStore.getEncoderWrapped());
    }

    // units: degrees per second
    float TurretSubsystem::getVelocity(aruwlib::motor::DjiMotor *motor) {
        return 360 * motor->getShaftRPM() / 60;
    }

    void TurretSubsystem::updateTurretVals() {
        pitchMotor.setDesiredOutput(pitchMotorPid.getValue());
        yawMotor.setDesiredOutput(yawMotorPid.getValue());
    }

    void TurretSubsystem::updateCurrentTurretAngles()
    {
        if (yawMotor.isMotorOnline())
        {
            currYawAngle.setValue(ENCODER_TO_DEGREE(
                    yawMotor.encStore.getEncoderWrapped() - YAW_START_ENCODER_POSITION)
                    + TURRET_START_ANGLE);
        }
        else
        {
            currYawAngle.setValue(TURRET_START_ANGLE);
        }
        if (pitchMotor.isMotorOnline())
        {
            currPitchAngle.setValue(ENCODER_TO_DEGREE(
                    pitchMotor.encStore.getEncoderWrapped() - PITCH_START_ENCODER_POSITION)
                    + TURRET_START_ANGLE);
        }
        else
        {
            currPitchAngle.setValue(TURRET_START_ANGLE);
        }
    }

    void TurretSubsystem::updateDesiredTurretAngles(float newYawAngle, float newPitchAngle)
    {
        aruwlib::algorithms::limitVal<float>(
            newYawAngle, TURRET_YAW_MIN_ANGLE, TURRET_YAW_MAX_ANGLE);
        aruwlib::algorithms::limitVal<float>(
            newPitchAngle, TURRET_PITCH_MIN_ANGLE, TURRET_PITCH_MAX_ANGLE);
        
        desiredYawAngle.setValue(newYawAngle);
        desiredPitchAngle.setValue(newPitchAngle);
    }

    void TurretSubsystem::runTurretPositionPid()
    {
        yawMotorPid.update(currYawAngle.difference(desiredYawAngle));
        pitchMotorPid.update(currPitchAngle.difference(desiredPitchAngle));
        yawMotor.setDesiredOutput(yawMotorPid.getValue());
        pitchMotor.setDesiredOutput(pitchMotorPid.getValue());
    }

    float TurretSubsystem::getYawAngleFromCenter()
    {
        aruwlib::algorithms::ContiguousFloat yawAngleFromCenter(
            currYawAngle.getValue() - TURRET_START_ANGLE, -180.0f, 180.0f);
        return yawAngleFromCenter.getValue();
    }

    float TurretSubsystem::getYawWrapped()
    {
        return currYawAngle.getValue();
    }

    float TurretSubsystem::getPitchWrapped()
    {
        return currPitchAngle.getValue();
    }

}  // control

}  // aruwsrc