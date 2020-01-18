#ifndef __TURRET_SUBSYSTEM_HPP__
#define __TURRET_SUBSYSTEM_HPP__

#include <stdint.h>
#include "src/aruwlib/control/subsystem.hpp"
#include "src/aruwsrc/control/turret_cv_command.hpp"
#include "src/aruwsrc/control/turret_manual_command.hpp"
#include "src/aruwlib/motor/dji_motor.hpp"

using namespace aruwlib::control;

namespace aruwsrc
{

namespace control
{

class TurretSubsystem : public Subsystem {
friend TurretCVCommand;
friend TurretManualCommand;

 public:
    TurretSubsystem(
        aruwlib::motor::MotorId pitchMotorId = PITCH_MOTOR_ID,
        aruwlib::motor::MotorId yawMotorId = YAW_MOTOR_ID)
        {}
    
    void pitchMotorToDegree(uint32_t degrees);  

    void yawMotorToDegree(uint32_t degrees);

    void pitchMotorToRadian(uint32_t radians);

    void yawMotorToRadian(uint32_t radians);

    void setDefaultCommand();
    
    void refresh() {
        updateTurretVals();
    }
    
 private:
    enum turretMode
    {
        IDLE = 0,
        MANUAL,
        CV
    };
    
    turretMode turretStatus = turretMode::IDLE;
    
    static const aruwlib::motor::MotorId PITCH_MOTOR_ID;
    static const aruwlib::motor::MotorId YAW_MOTOR_ID;

    void goToDegree(aruwlib::motor::DjiMotor motor, uint32_t degrees);

    void goToRadian(aruwlib::motor::DjiMotor motor, uint32_t radians);

    void updateTurretVals();

};

}  // control

}  // aruwsrc

#endif
