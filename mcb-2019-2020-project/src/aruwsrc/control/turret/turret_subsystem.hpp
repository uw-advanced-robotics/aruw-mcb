#ifndef __TURRET_SUBSYSTEM_HPP__
#define __TURRET_SUBSYSTEM_HPP__

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

    float getYawVelocity() const;

    float getPitchVelocity() const;

    float getYawAngleFromCenter() const;

    float getPitchAngleFromCenter() const;

    void refresh();

    void setPitchMotorOutput(float out);

    void setYawMotorOutput(float out);

    bool isTurretOnline() const;

    float getYawAngle() const;

    float getPitchAngle() const;

 private:
    const int TURRET_YAW_MIN_ANGLE = 0.0f;
    const int TURRET_YAW_MAX_ANGLE = 180.0f;
    const int TURRET_PITCH_MIN_ANGLE = 75.0f;
    const int TURRET_PITCH_MAX_ANGLE = 110.0f;
    const int TURRET_START_ANGLE = 90.0f;

    const int YAW_START_ENCODER_POSITION = 8160;
    const int PITCH_START_ENCODER_POSITION = 4780;

    const aruwlib::can::CanBus CAN_BUS_MOTORS = aruwlib::can::CanBus::CAN_BUS1;
    static const aruwlib::motor::MotorId PITCH_MOTOR_ID = aruwlib::motor::MOTOR6;
    static const aruwlib::motor::MotorId YAW_MOTOR_ID = aruwlib::motor::MOTOR5;

    aruwlib::motor::DjiMotor pitchMotor;
    aruwlib::motor::DjiMotor yawMotor;

    aruwlib::algorithms::ContiguousFloat currYawAngle;
    aruwlib::algorithms::ContiguousFloat currPitchAngle;

    void updateCurrentTurretAngles();

    float getVelocity(const aruwlib::motor::DjiMotor &motor) const;
};

}  // namespace control

}  // namespace aruwsrc

#endif
