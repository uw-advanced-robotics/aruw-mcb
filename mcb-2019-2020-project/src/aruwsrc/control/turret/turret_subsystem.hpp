#ifndef __TURRET_SUBSYSTEM_HPP__
#define __TURRET_SUBSYSTEM_HPP__

#include <stdint.h>
#include <modm/math/filter/pid.hpp>
#include "src/aruwlib/control/subsystem.hpp"
#include "src/aruwlib/motor/dji_motor.hpp"
#include "turret_manual_command.hpp"
#include "turret_cv_command.hpp"

using namespace aruwlib::control;

namespace aruwsrc
{

namespace control
{

class TurretSubsystem : public Subsystem {

friend class TurretCVCommand;
friend class TurretManualCommand;

 public:
    TurretSubsystem(void);

    void setPitchVelocity(float velocity);

    void setYawVelocity(float velocity);
    
    void pitchMotorToDegree(float degrees);  

    void yawMotorToDegree(float degrees);

    void incPitchMotorByDegree(float degrees);  

    void incYawMotorByDegree(float degrees);
    
    float getYawAngle(void);

    float getPitchAngle(void);

    float getYawVelocity(void);

    float getPitchVelocity(void);

    void refresh(void);

    aruwlib::motor::DjiMotor pitchMotor;
    aruwlib::motor::DjiMotor yawMotor;

 private:
    const int TURRET_ROTATION_BOUNDS = 90;
    const int YAW_START_POSITION = 4090;
    const int PITCH_START_POSITION = -4090;

    const aruwlib::can::CanBus CAN_BUS_MOTORS = aruwlib::can::CanBus::CAN_BUS1;
    static const aruwlib::motor::MotorId PITCH_MOTOR_ID = aruwlib::motor::MOTOR7;
    static const aruwlib::motor::MotorId YAW_MOTOR_ID = aruwlib::motor::MOTOR8;

    TurretManualCommand *turretManual;
    TurretCVCommand *turretCV;

    enum turretMode
    {
        IDLE = 0,
        MANUAL,
        CV
    };

    turretMode turretStatus;
    
    modm::Pid<float> pitchMotorPid;
    modm::Pid<float> yawMotorPid;

    float yawTarget;
    float pitchTarget;

    float convertToUnwrappedEncoder(aruwlib::motor::DjiMotor *motor, float degrees);

    float getVelocity(aruwlib::motor::DjiMotor *motor);
    float getAngle(aruwlib::motor::DjiMotor *motor);

    void updateTurretVals(void);
};

}  // control

}  // aruwsrc

#endif
