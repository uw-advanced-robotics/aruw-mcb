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
 public:
    TurretSubsystem();

    void pitchMotorToDegree(float degrees);

    void yawMotorToDegree(float degrees);

    void incPitchMotorByDegree(float degrees);

    void incYawMotorByDegree(float degrees);

    float getYawAngle();

    float getPitchAngle();

    float getYawVelocity();

    float getPitchVelocity();

    void refresh();

    void setPitchMotorOutput(float out);

    void setYawMotorOutput(float out);

 private:
    const int TURRET_YAW_MIN_ANGLE = 0;
    const int TURRET_YAW_MAX_ANGLE = 180;
    const int TURRET_PITCH_MIN_ANGLE = 75;
    const int TURRET_PITCH_MAX_ANGLE = 110;
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

    aruwlib::motor::DjiMotor pitchMotor;
    aruwlib::motor::DjiMotor yawMotor;

    float getVelocity(const aruwlib::motor::DjiMotor &motor);
    float getAngle(const aruwlib::motor::DjiMotor &motor);
};

}  // namespace control

}  // namespace aruwsrc

#endif
