#include <algorithm>
#include <random>
#include "turret_subsystem.hpp"

#define DEGREE_TO_ENCODER(degree) ((8192 * degree) / 360)
#define ENCODER_TO_DEGREE(encoder) ((encoder * 360) / 8192)

namespace aruwsrc
{

namespace control
{
    TurretSubsystem::TurretSubsystem() : 
        pitchMotor(PITCH_MOTOR_ID, CAN_BUS_MOTORS, true),
        yawMotor(YAW_MOTOR_ID, CAN_BUS_MOTORS, false),
        turretStatus(IDLE) {
        turretManual = new aruwsrc::control::TurretManualCommand(this);
        turretCV = new aruwsrc::control::TurretCVCommand(this);
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
}  // control

}  // aruwsrc