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
    #if defined(TARGET_SOLDIER)
    static constexpr float TURRET_YAW_MIN_ANGLE = TURRET_START_ANGLE - 90.0f;
    static constexpr float TURRET_YAW_MAX_ANGLE = TURRET_START_ANGLE + 90.0f;
    static constexpr float TURRET_PITCH_MIN_ANGLE = TURRET_START_ANGLE - 13.0f;
    static constexpr float TURRET_PITCH_MAX_ANGLE = TURRET_START_ANGLE + 20.0f;
    #elif defined(TARGET_DRONE)
    static constexpr float TURRET_YAW_MIN_ANGLE = TURRET_START_ANGLE - 60.0f;
    static constexpr float TURRET_YAW_MAX_ANGLE = TURRET_START_ANGLE + 60.0f;
    static constexpr float TURRET_PITCH_MIN_ANGLE = TURRET_START_ANGLE - 13.0f;
    static constexpr float TURRET_PITCH_MAX_ANGLE = TURRET_START_ANGLE + 20.0f;
    #else
    static constexpr float TURRET_YAW_MIN_ANGLE = TURRET_START_ANGLE - 90.0f;
    static constexpr float TURRET_YAW_MAX_ANGLE = TURRET_START_ANGLE + 90.0f;
    static constexpr float TURRET_PITCH_MIN_ANGLE = TURRET_START_ANGLE - 10.0f;
    static constexpr float TURRET_PITCH_MAX_ANGLE = TURRET_START_ANGLE + 10.0f;
    #endif

    TurretSubsystem();

    void refresh();

    bool isTurretOnline() const;

    int32_t getYawVelocity() const;
    int32_t getPitchVelocity() const;

    float getYawAngleFromCenter() const;
    float getPitchAngleFromCenter() const;

    const aruwlib::algorithms::ContiguousFloat& getYawAngle();
    const aruwlib::algorithms::ContiguousFloat& getPitchAngle();

    void setYawMotorOutput(float out);
    void setPitchMotorOutput(float out);

    void updatePrevYawTarget(const float& yaw);
    void updatePrevPitchTarget(const float& pitch);

    /**
     * Calculates a yaw output that uses the desired chassis rotation as a feed forward gain.
     * 
     * The chassis rotation is given in desired wheel rpm.
     */
    float yawFeedForwardCalculation(float desiredChassisRotation);

    const aruwlib::algorithms::ContiguousFloat& getPrevYawTarget() const;
    const aruwlib::algorithms::ContiguousFloat& getPrevPitchTarget() const;

    const aruwlib::algorithms::ContiguousFloat& getYawTarget() const;
    const aruwlib::algorithms::ContiguousFloat& getPitchTarget() const;

    void setYawTarget(float target);
    void setPitchTarget(float target);

    static float projectChassisRelativeYawToWorldRelative(float yawAngle, float imuInitialAngle);

    static float projectWorldRelativeYawToChassisFrame(float yawAngle, float imuInitialAngle);

 private:
    const uint16_t YAW_START_ENCODER_POSITION = 8160;
    const uint16_t PITCH_START_ENCODER_POSITION = 4100;

    static constexpr float FEED_FORWARD_KP = 2.75f;
    static constexpr float FEED_FORWARD_SIN_GAIN = 1.0f;
    static constexpr float FEED_FORWARD_KD = 30.0f;
    static constexpr float FEED_FORWARD_MAX_OUTPUT = 20000.0f;
    static constexpr float FEED_FORWARD_DERIVATIVE_LOW_PASS = 0.154f;

    const aruwlib::can::CanBus CAN_BUS_MOTORS = aruwlib::can::CanBus::CAN_BUS1;
    static const aruwlib::motor::MotorId PITCH_MOTOR_ID = aruwlib::motor::MOTOR6;
    static const aruwlib::motor::MotorId YAW_MOTOR_ID = aruwlib::motor::MOTOR5;

    float feedforwardChassisRotateDerivative = 0.0f;
    float feedforwardPrevChassisRotationDesired = 0.0f;

    aruwlib::motor::DjiMotor pitchMotor;
    aruwlib::motor::DjiMotor yawMotor;

    aruwlib::algorithms::ContiguousFloat currPitchAngle;
    aruwlib::algorithms::ContiguousFloat currYawAngle;

    aruwlib::algorithms::ContiguousFloat yawTarget;
    aruwlib::algorithms::ContiguousFloat pitchTarget;

    aruwlib::algorithms::ContiguousFloat prevYawTarget;
    aruwlib::algorithms::ContiguousFloat pretPitchTarget;

    void updateCurrentTurretAngles();

    void updateCurrentYawAngle();

    void updateCurrentPitchAngle();

    int32_t getVelocity(const aruwlib::motor::DjiMotor &motor) const;
};

}  // namespace control

}  // namespace aruwsrc

#endif
