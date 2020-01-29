#ifndef __TURRET_SUBSYSTEM_HPP__
#define __TURRET_SUBSYSTEM_HPP__

#include <stdint.h>
#include "src/aruwlib/control/subsystem.hpp"
#include "src/aruwlib/motor/dji_motor.hpp"
//#include "src/aruwsrc/control/turret_cv_command.hpp"
#include "turret_manual_command.hpp"

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
    
    void pitchMotorToDegree(uint32_t degrees);  

    void yawMotorToDegree(uint32_t degrees);

    void incPitchMotorByDegree(int32_t degrees);  

    void incYawMotorByDegree(int32_t degrees);
    
    void refresh(void);

 private:
    const int TURRET_ROTATION_BOUNDS = 90;
    const aruwlib::can::CanBus CAN_BUS_MOTORS = aruwlib::can::CanBus::CAN_BUS1;

    static const aruwlib::motor::MotorId PITCH_MOTOR_ID = aruwlib::motor::MOTOR1;
    static const aruwlib::motor::MotorId YAW_MOTOR_ID = aruwlib::motor::MOTOR2;

    enum turretMode
    {
        IDLE = 0,
        MANUAL,
        CV
    };

    turretMode turretStatus;
    
    modm::Pid<float> pitchMotorPid;
    modm::Pid<float> yawMotorPid;

    aruwlib::motor::DjiMotor *pitchMotor;
    aruwlib::motor::DjiMotor *yawMotor;

    void goToDegree(aruwlib::motor::DjiMotor *motor, int32_t degrees);

    void incByDegree(aruwlib::motor::DjiMotor *motor, int32_t degrees);

    void updateTurretVals(void);

    template<class T>
    constexpr const T& clamp( const T& v, const T& lo, const T& hi )
    {
        return (v < lo) ? lo : (hi < v) ? hi : v;
    }
};

}  // control

}  // aruwsrc

#endif
