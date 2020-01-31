#include <algorithm>
#include <random>
#include "turret_subsystem.hpp"
#include "turret_manual_command.hpp"

#define PI 3.141592
#define DEGREE_TO_ENCODER(degree) ((8192 * degree) / 360)
#define ENCODER_TO_DEGREE(encoder) ((encoder * 360) / 8192)

namespace aruwsrc
{

namespace control
{
    TurretSubsystem::TurretSubsystem() : 
        pitchMotor(PITCH_MOTOR_ID, CAN_BUS_MOTORS, true),
        yawMotor(YAW_MOTOR_ID, CAN_BUS_MOTORS, false) {
        turretStatus = IDLE;
        modm::SmartPointer turretManualCommand(
        new aruwsrc::control::TurretManualCommand(this));
        setDefaultCommand(turretManualCommand);
        pitchMotor.encStore.getEncoderUnwrapped();
        pitchEncoderTarget = PITCH_START_POSITION;
        yawEncoderTarget = YAW_START_POSITION;
    }

    void TurretSubsystem::pitchMotorToDegree(uint32_t degrees) {
        pitchEncoderTarget = getDegree(&pitchMotor, degrees);
    }

    void TurretSubsystem::yawMotorToDegree(uint32_t degrees) {
        yawEncoderTarget = getDegree(&yawMotor, degrees);
    }

    void TurretSubsystem::incPitchMotorByDegree(int32_t degrees) {
        pitchEncoderTarget += DEGREE_TO_ENCODER(degrees);
    }

    void TurretSubsystem::incYawMotorByDegree(int32_t degrees) {
        yawEncoderTarget += DEGREE_TO_ENCODER(degrees);
    }

    void TurretSubsystem::refresh() {
        updateTurretVals();
    }

    int32_t TurretSubsystem::getDegree(aruwlib::motor::DjiMotor *motor, int32_t degrees) {
        int32_t targetDegree = degrees < 0 ? 360 - (degrees % 360) : degrees % 360;
        float relativePosition = fmod(DEGREE_TO_ENCODER(targetDegree) - motor->encStore.getEncoderWrapped(), 8192);
        return motor->encStore.getEncoderUnwrapped() + DEGREE_TO_ENCODER(relativePosition);
    }

    void TurretSubsystem::updateTurretVals() {
        int i = pitchMotor.encStore.getEncoderUnwrapped();
        printf("%d",i);
        pitchMotorPid.update(pitchEncoderTarget - pitchMotor.encStore.getEncoderUnwrapped());
        yawMotorPid.update(yawEncoderTarget - yawMotor.encStore.getEncoderUnwrapped());
        pitchMotor.setDesiredOutput(pitchMotorPid.getValue());
        yawMotor.setDesiredOutput(yawMotorPid.getValue());
    }
}  // control

}  // aruwsrc