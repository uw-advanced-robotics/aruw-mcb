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
        turretStatus(INIT),
        pitchMotor(PITCH_MOTOR_ID, CAN_BUS_MOTORS, true),
        yawMotor(YAW_MOTOR_ID, CAN_BUS_MOTORS, false),
        currYawAngle(0.0f, 0.0f, 360.0f),
        currPitchAngle(0.0f, 0.0f, 360.0f)
    {
        turretInit = new aruwsrc::control::TurretInitCommand(this);
        turretManual = new aruwsrc::control::TurretManualCommand(this);
        turretCV = new aruwsrc::control::TurretCVCommand(this);
        setDefaultCommand(modm::SmartPointer(turretInit));
        IoMapper::addHoldMapping(IoMapper::newKeyMap(Remote::Switch::LEFT_SWITCH,
                                                     Remote::SwitchState::UP, {}),
                                                     modm::SmartPointer(turretCV));
    }

    float TurretSubsystem::getYawAngleFromCenter()
    {
        aruwlib::algorithms::ContiguousFloat yawAngleFromCenter(
            currYawAngle.getValue() - TURRET_START_ANGLE, -180.0f, 180.0f);
        return yawAngleFromCenter.getValue();
    }

    float TurretSubsystem::getPitchAngleFromCenter()
    {
        aruwlib::algorithms::ContiguousFloat yawAngleFromCenter(
            currPitchAngle.getValue() - TURRET_START_ANGLE, -180.0f, 180.0f);
        return yawAngleFromCenter.getValue();
    }

    float TurretSubsystem::getYawVelocity(void) {
        return getVelocity(yawMotor);
    }

    float TurretSubsystem::getPitchVelocity(void) {
        return getVelocity(pitchMotor);
    }

    float TurretSubsystem::getAngle(const aruwlib::motor::DjiMotor &motor) {
        return ENCODER_TO_DEGREE(motor.encStore.getEncoderWrapped());
    }

    // units: degrees per second
    float TurretSubsystem::getVelocity(const aruwlib::motor::DjiMotor &motor) {
        return 360 * motor.getShaftRPM() / 60;
    }

    void TurretSubsystem::refresh() {
        switch (turretStatus) {
            case INIT:
                if (turretInit->isFinished()) {
                    setDefaultCommand(modm::SmartPointer(turretManual));
                    turretStatus = MANUAL;
                }
                break;
            default:
                updateCurrentTurretAngles();
        }
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

    void TurretSubsystem::setPitchMotorOutput(float out) {
        float angle = getPitchAngleFromCenter();
        if ((angle > TURRET_PITCH_MAX_ANGLE && out > 0) ||
            (angle < TURRET_PITCH_MIN_ANGLE && out < 0)) {
            pitchMotor.setDesiredOutput(0);
        } else {
            pitchMotor.setDesiredOutput(out);
        }
    }

    void TurretSubsystem::setYawMotorOutput(float out) {
        float angle = getYawAngleFromCenter();
        if ((angle > TURRET_YAW_MAX_ANGLE && out < 0) ||
            (angle < TURRET_YAW_MIN_ANGLE && out > 0)) {
            yawMotor.setDesiredOutput(0);
        } else {
            yawMotor.setDesiredOutput(out);
        }
    }
}  // namespace control

}  // namespace aruwsrc
