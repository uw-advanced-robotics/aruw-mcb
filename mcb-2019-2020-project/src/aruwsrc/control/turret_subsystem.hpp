#ifndef __TURRET_SUBSYSTEM_HPP__
#define __TURRET_SUBSYSTEM_HPP__

#include <stdint.h>
#include "src/aruwlib/control/subsystem.hpp"
#include "src/aruwlib/motor/dji_motor.hpp"
//#include "src/aruwsrc/control/turret_cv_command.hpp"
#include "src/aruwsrc/control/turret_manual_command.hpp"

using namespace aruwlib::control;

namespace aruwsrc
{

namespace control
{

class TurretSubsystem : public Subsystem {

friend class TurretCVCommand;
friend class TurretManualCommand;

 public:
    TurretSubsystem(
        aruwlib::motor::MotorId pitchMotorId = PITCH_MOTOR_ID,
        aruwlib::motor::MotorId yawMotorId = YAW_MOTOR_ID) {
        pitchMotor = aruwlib::motor::DjiMotor(pitchMotorId, CAN_BUS_MOTORS, true);
        yawMotor = aruwlib::motor::DjiMotor(yawMotorId, CAN_BUS_MOTORS, false); 
        turretStatus = IDLE; }
    
    static void pitchMotorToDegree(uint32_t degrees);  

    static void yawMotorToDegree(uint32_t degrees);

    static void incPitchMotorByDegree(int32_t degrees);  

    static void incYawMotorByDegree(int32_t degrees);
    
    void refresh(void);
    
 private:
    const int TURRET_ROTATION_BOUNDS = 90;
    const aruwlib::can::CanBus CAN_BUS_MOTORS = aruwlib::can::CanBus::CAN_BUS1;

    static const aruwlib::motor::MotorId PITCH_MOTOR_ID;
    static const aruwlib::motor::MotorId YAW_MOTOR_ID;

    enum turretMode
    {
        IDLE = 0,
        MANUAL,
        CV
    };

    static turretMode turretStatus;
    
    static modm::Pid<float> pitchMotorPid;
    static modm::Pid<float> yawMotorPid;

    static aruwlib::motor::DjiMotor pitchMotor;
    static aruwlib::motor::DjiMotor yawMotor;

    static void goToDegree(aruwlib::motor::DjiMotor *motor, int32_t degrees);

    static void incByDegree(aruwlib::motor::DjiMotor *motor, int32_t degrees);

    static void setTurretIdle() {turretStatus = IDLE; }
    static void setTurretManual() {turretStatus = MANUAL; }
    static void setTurretCV() {turretStatus = CV; }

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
