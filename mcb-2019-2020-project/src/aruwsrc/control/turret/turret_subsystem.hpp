#ifndef __TURRET_SUBSYSTEM_HPP__
#define __TURRET_SUBSYSTEM_HPP__

#include <modm/math/filter/pid.hpp>
#include "src/aruwlib/control/subsystem.hpp"
#include "src/aruwlib/motor/dji_motor.hpp"
#include "src/aruwlib/algorithms/contiguous_float.hpp"

using namespace aruwlib::control;

namespace aruwsrc
{

namespace control
{

class TurretSubsystem : public Subsystem {
 public:
    static constexpr float TURRET_START_ANGLE = 90.0f;
    const float TURRET_YAW_MIN_ANGLE = 0.0f;
    const float TURRET_YAW_MAX_ANGLE = 180.0f;
    const float TURRET_PITCH_MIN_ANGLE = 75.0f;
    const float TURRET_PITCH_MAX_ANGLE = 110.0f;

    TurretSubsystem();

    float getYawVelocity() const;

    float getPitchVelocity() const;

    float getYawAngleFromCenter() const;

    float getPitchAngleFromCenter() const;

    void refresh();

    void setPitchMotorOutput(float out);

    void setYawMotorOutput(float out);

    bool isTurretOnline() const;

    float getRemoteXMovement();

    float getRemoteYMovement();

    const aruwlib::algorithms::ContiguousFloat& getYawAngle() const;

    const aruwlib::algorithms::ContiguousFloat& getPitchAngle() const;


    void updateCurrentTurretAngles();

 private:
    const float YAW_START_ENCODER_POSITION = 8160;
    /// \todo fix this encoder starting position
    const float PITCH_START_ENCODER_POSITION = 4130;

    const float REMOTE_INPUT_SCALER = 30000;

    const aruwlib::can::CanBus CAN_BUS_MOTORS = aruwlib::can::CanBus::CAN_BUS1;
    static const aruwlib::motor::MotorId PITCH_MOTOR_ID = aruwlib::motor::MOTOR6;
    static const aruwlib::motor::MotorId YAW_MOTOR_ID = aruwlib::motor::MOTOR5;

    aruwlib::motor::DjiMotor pitchMotor;
    aruwlib::motor::DjiMotor yawMotor;

    aruwlib::algorithms::ContiguousFloat currYawAngle;
    aruwlib::algorithms::ContiguousFloat currPitchAngle;

    float getVelocity(const aruwlib::motor::DjiMotor &motor) const;
};

}  // namespace control

}  // namespace aruwsrc

#endif
