#include <algorithm>
#include <random>
#include "turret_subsystem.hpp"

#define PI 3.141592
#define DEGREE_TO_ENCODER(degree) ((8192 * degree) / 360)
#define ENCODER_TO_DEGREE(encoder) ((encoder * 360) / 8192)

namespace aruwsrc
{

namespace control
{
    const aruwlib::motor::MotorId PITCH_MOTOR_ID = aruwlib::motor::MOTOR1;
    const aruwlib::motor::MotorId YAW_MOTOR_ID = aruwlib::motor::MOTOR2;
    aruwsrc::control::TurretManualCommand turretManualCommand;
    //aruwsrc::control::TurretCVCommand turretCVCommand;

    void TurretSubsystem::pitchMotorToDegree(uint32_t degrees) {
        goToDegree(&pitchMotor, degrees);
    }

    void TurretSubsystem::yawMotorToDegree(uint32_t degrees) {
        goToDegree(&yawMotor, degrees);
    }

    void TurretSubsystem::incPitchMotorByDegree(int32_t degrees) {
        incByDegree(&pitchMotor, degrees);
    }

    void TurretSubsystem::incYawMotorByDegree(int32_t degrees) {
        incByDegree(&yawMotor, degrees);
    }

    void TurretSubsystem::refresh() {
        updateTurretVals();
    }

    void TurretSubsystem::goToDegree(aruwlib::motor::DjiMotor *motor, int32_t degrees) {
        int32_t targetDegree = degrees < 0 ? 360 - (degrees % 360) : degrees % 360;
        float relativePosition = fmod(DEGREE_TO_ENCODER(targetDegree) - motor->encStore.getEncoderWrapped(), 8192);
        motor->setDesiredOutput(motor->encStore.getEncoderUnwrapped() + DEGREE_TO_ENCODER(relativePosition));
    }

    void TurretSubsystem::incByDegree(aruwlib::motor::DjiMotor *motor, int32_t degrees) {
        motor->setDesiredOutput(motor->getOutputDesired() + DEGREE_TO_ENCODER(degrees));
    }

    void TurretSubsystem::updateTurretVals() {
        pitchMotor.setDesiredOutput(clamp<uint16_t>(pitchMotor.getOutputDesired(), -TURRET_ROTATION_BOUNDS, TURRET_ROTATION_BOUNDS));
        yawMotor.setDesiredOutput(clamp<uint16_t>(yawMotor.getOutputDesired(), -TURRET_ROTATION_BOUNDS, TURRET_ROTATION_BOUNDS));
        pitchMotor.setDesiredOutput(pitchMotor.getOutputDesired() - pitchMotor.encStore.getEncoderUnwrapped());
        yawMotor.setDesiredOutput(yawMotor.getOutputDesired() - yawMotor.encStore.getEncoderUnwrapped());
    }

}  // control

}  // aruwsrc