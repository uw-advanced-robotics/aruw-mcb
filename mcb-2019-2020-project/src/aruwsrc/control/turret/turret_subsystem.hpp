#ifndef __TURRET_SUBSYSTEM_HPP__
#define __TURRET_SUBSYSTEM_HPP__

#include <stdint.h>
#include <modm/math/filter/pid.hpp>
#include "src/aruwlib/control/subsystem.hpp"
#include "src/aruwlib/motor/dji_motor.hpp"
#include "turret_manual_command.hpp"
#include "turret_cv_command.hpp"

#include "src/aruwlib/algorithms/contiguous_float.hpp"

using namespace aruwlib::control;

namespace aruwsrc
{

namespace control
{

class TurretSubsystem : public Subsystem {

friend class TurretCVCommand;
friend class TurretManualCommand;

 public:
    TurretSubsystem();

    void setPitchVelocity(float velocity);

    void setYawVelocity(float velocity);
    
    void pitchMotorToDegree(float degrees);  

    void yawMotorToDegree(float degrees);

    void incPitchMotorByDegree(float degrees);  

    void incYawMotorByDegree(float degrees);
    
    float getYawAngle();

    float getPitchAngle();

    float getYawVelocity();

    float getPitchVelocity();

    void refresh();

    aruwlib::motor::DjiMotor pitchMotor;
    aruwlib::motor::DjiMotor yawMotor;

    void updateCurrentTurretAngles();

    void updateDesiredTurretAngles(float newYawAngle, float newPitchAngle);

    void runTurretPositionPid();

    void updateTurretVals();

    float getYawWrapped();

    float getPitchWrapped();

    float getYawAngleFromCenter();

 private:
    const int TURRET_YAW_MIN_ANGLE = 0;
    const int TURRET_YAW_MAX_ANGLE = 180;
    const int TURRET_PITCH_MIN_ANGLE = 80;
    const int TURRET_PITCH_MAX_ANGLE = 100;
    const int TURRET_START_ANGLE = 90;

    const int YAW_START_ENCODER_POSITION = 8160;
    const int PITCH_START_ENCODER_POSITION = 4780;

    const aruwlib::can::CanBus CAN_BUS_MOTORS = aruwlib::can::CanBus::CAN_BUS1;
    static const aruwlib::motor::MotorId PITCH_MOTOR_ID = aruwlib::motor::MOTOR6;
    static const aruwlib::motor::MotorId YAW_MOTOR_ID = aruwlib::motor::MOTOR5;
    
    TurretManualCommand *turretManual;
    TurretCVCommand *turretCV;

    enum turretMode
    {
        IDLE = 0,
        MANUAL,
        CV
    };

    turretMode turretStatus;

    const float TURRET_YAW_PID_P = 3000.0f;
    const float TURRET_YAW_PID_I = 0.0f;
    const float TURRET_YAW_PID_D = 18000.0f;

    const float TURRET_PITCH_PID_P = 1500.0f;
    const float TURRET_PITCH_PID_I = 0.0f;
    const float TURRET_PITCH_PID_D = 6000.0f;

    const float TURRET_MAX_VOLTAGE_OUT = 15000.0f;

    modm::Pid<float> pitchMotorPid;
    modm::Pid<float> yawMotorPid;

    float yawTarget;
    float pitchTarget;

    aruwlib::algorithms::ContiguousFloat currYawAngle;
    aruwlib::algorithms::ContiguousFloat currPitchAngle;

    aruwlib::algorithms::ContiguousFloat desiredYawAngle;
    aruwlib::algorithms::ContiguousFloat desiredPitchAngle;

    float convertToUnwrappedEncoder(aruwlib::motor::DjiMotor *motor, float degrees);

    float getVelocity(aruwlib::motor::DjiMotor *motor);
    float getAngle(aruwlib::motor::DjiMotor *motor);
};

}  // control

}  // aruwsrc

#endif
