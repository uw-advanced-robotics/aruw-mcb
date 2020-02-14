#ifndef __TURRET_SUBSYSTEM_HPP__
#define __TURRET_SUBSYSTEM_HPP__

#include <stdint.h>
#include <modm/math/filter/pid.hpp>
#include "src/aruwlib/control/subsystem.hpp"
#include "src/aruwlib/motor/dji_motor.hpp"
#include "turret_manual_command.hpp"
#include "turret_cv_command.hpp"
#include "turret_init_command.hpp"

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

    float getYawVelocity();

    float getPitchVelocity();

    float getYawAngleFromCenter();

    float getPitchAngleFromCenter();

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

    uint16_t YAW_P = 1.0f;
    uint16_t YAW_I = 0.0f;
    uint16_t YAW_D = 0.0f;
    uint16_t YAW_MAX_ERROR_SUM = 0.0f;
    uint16_t YAW_MAX_OUTPUT = 16000;

    uint16_t PITCH_P = 1.0f;
    uint16_t PITCH_I = 0.0f;
    uint16_t PITCH_D = 0.0f;
    uint16_t PITCH_MAX_ERROR_SUM = 0.0f;
    uint16_t PITCH_MAX_OUTPUT = 16000;

    const aruwlib::can::CanBus CAN_BUS_MOTORS = aruwlib::can::CanBus::CAN_BUS1;
    static const aruwlib::motor::MotorId PITCH_MOTOR_ID = aruwlib::motor::MOTOR6;
    static const aruwlib::motor::MotorId YAW_MOTOR_ID = aruwlib::motor::MOTOR5;

    TurretManualCommand *turretManual;
    TurretCVCommand *turretCV;
    TurretInitCommand *turretInit;

    enum turretMode
    {
        INIT = 0,
        MANUAL,
        CV
    };

    turretMode turretStatus;

    modm::Pid<float> YawPid;
    modm::Pid<float> PitchPid;

    aruwlib::motor::DjiMotor pitchMotor;
    aruwlib::motor::DjiMotor yawMotor;

    aruwlib::algorithms::ContiguousFloat currYawAngle;
    aruwlib::algorithms::ContiguousFloat currPitchAngle;

    void updateCurrentTurretAngles();

    float getVelocity(const aruwlib::motor::DjiMotor &motor);
    float getAngle(const aruwlib::motor::DjiMotor &motor);
};

}  // namespace control

}  // namespace aruwsrc

#endif
