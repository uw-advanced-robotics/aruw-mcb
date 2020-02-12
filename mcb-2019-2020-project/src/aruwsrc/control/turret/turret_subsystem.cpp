#include <algorithm>
#include <random>
#include "turret_subsystem.hpp"
#include "src/aruwlib/algorithms/math_user_utils.hpp"
#include "src/aruwlib/control/controller_mapper.hpp"

#define DEGREE_TO_ENCODER(degree) ((8192 * degree) / 360)
#define ENCODER_TO_DEGREE(encoder) ((static_cast<float>(encoder) * 360.0f) / 8192.0f)

namespace aruwsrc
{

namespace control
{
    TurretSubsystem::TurretSubsystem() :
        turretStatus(IDLE),
        pitchMotor(PITCH_MOTOR_ID, CAN_BUS_MOTORS, true),
        yawMotor(YAW_MOTOR_ID, CAN_BUS_MOTORS, false) 
    {
        turretManual = new aruwsrc::control::TurretManualCommand(*this);
        turretCV = new aruwsrc::control::TurretCVCommand(*this);
        setDefaultCommand(modm::SmartPointer(turretManual));
        turretStatus = MANUAL;
        IoMapper::addHoldMapping(IoMapper::newKeyMap(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP, {}), modm::SmartPointer(turretCV));
    }

    float TurretSubsystem::getYawAngle(void) {
        return getAngle(yawMotor);
    }

    float TurretSubsystem::getPitchAngle(void) {
        return getAngle(pitchMotor);
    }

    float TurretSubsystem::getYawVelocity(void) {
        return getVelocity(yawMotor);
    }

    float TurretSubsystem::getPitchVelocity(void) {
        return getVelocity(pitchMotor);
    }

    float TurretSubsystem::getAngle(aruwlib::motor::DjiMotor &motor) {
        return motor.encStore.getEncoderWrapped();
    }

    // units: degrees per second
    float TurretSubsystem::getVelocity(aruwlib::motor::DjiMotor &motor) {
        return 360 * motor.getShaftRPM() / 60;
    }

    void TurretSubsystem::refresh() {
        
    }

    void TurretSubsystem::setPitchMotorOutput(float out) {
        pitchMotor.setDesiredOutput(out);
    }

    void TurretSubsystem::setYawMotorOutput(float out) {
        yawMotor.setDesiredOutput(out);
    }

}  // control

}  // aruwsrc