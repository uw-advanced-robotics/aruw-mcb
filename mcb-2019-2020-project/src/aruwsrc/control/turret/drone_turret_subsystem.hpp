#ifndef __DRONE_TURRET_SUBSYSTEM_HPP__
#define __DRONE_TURRET_SUBSYSTEM_HPP__

#include <modm/math/filter/pid.hpp>
#include "src/aruwlib/control/subsystem.hpp"
#include "src/aruwlib/motor/dji_motor.hpp"
#include "src/aruwlib/algorithms/contiguous_float.hpp"
#include "src/aruwlib/algorithms/linear_interpolation.hpp"

using namespace aruwlib::control;

namespace aruwsrc
{

namespace turret
{

class DroneTurretSubsystem : public Subsystem {
 public:
    static constexpr float TURRET_YAW_START_ANGLE = 90.0f;
    static constexpr float TURRET_YAW_MIN_ANGLE = TURRET_YAW_START_ANGLE - 55.0f;
    static constexpr float TURRET_YAW_MAX_ANGLE = TURRET_YAW_START_ANGLE + 55.0f;
    static constexpr float TURRET_PITCH_START_ANGLE = 90.0f;
    static constexpr float TURRET_PITCH_MIN_ANGLE = TURRET_PITCH_START_ANGLE - 45.0f;
    static constexpr float TURRET_PITCH_MAX_ANGLE = TURRET_PITCH_START_ANGLE + 55.0f;

    DroneTurretSubsystem();

    void refresh();

    bool isTurretOnline() const;

    int32_t getYawVelocity() const;
    int32_t getPitchVelocity() const;

    float getYawAngleFromCenter() const;
    float getPitchAngleFromCenter() const;

    const aruwlib::algorithms::ContiguousFloat& getYawAngle() const;
    const aruwlib::algorithms::ContiguousFloat& getPitchAngle() const;

    void setYawMotorOutput(float out);
    void setPitchMotorOutput(float out);

    /**
     * Calculates a yaw output that uses the desired chassis rotation as a feed forward gain.
     * 
     * The chassis rotation is given in desired wheel rpm.
     */
    float yawFeedForwardCalculation();
    float pitchFeedForwardCalculation();

    /**
     * Set a target angle in chassis frame, the angle is accordingly limited.
     * Note that since there is no controller in this subsystem, this target
     * angle merely acts as a safe way to set angles when using a position controller.
     */
    void setYawTarget(float target);
    void setPitchTarget(float target);

    float getYawTarget() const;
    float getPitchTarget() const;

    void updateCurrentTurretAngles();

 private:
    const uint16_t YAW_START_ENCODER_POSITION = 1800;
    const uint16_t PITCH_START_ENCODER_POSITION = 1356;

    const aruwlib::can::CanBus CAN_BUS_MOTORS = aruwlib::can::CanBus::CAN_BUS1;
    static const aruwlib::motor::MotorId PITCH_MOTOR_ID = aruwlib::motor::MOTOR6;
    static const aruwlib::motor::MotorId YAW_MOTOR_ID = aruwlib::motor::MOTOR5;

    aruwlib::motor::DjiMotor pitchMotor;
    aruwlib::motor::DjiMotor yawMotor;

    aruwlib::algorithms::ContiguousFloat currPitchAngle;
    aruwlib::algorithms::ContiguousFloat currYawAngle;

    aruwlib::algorithms::ContiguousFloat yawTarget;
    aruwlib::algorithms::ContiguousFloat pitchTarget;

    void updateCurrentYawAngle();
    void updateCurrentPitchAngle();

    int32_t getVelocity(const aruwlib::motor::DjiMotor &motor) const;
};


}
}

#endif
