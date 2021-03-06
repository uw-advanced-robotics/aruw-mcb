#ifndef ENGINEER_WRIST_SUBSYSTEM_HPP_
#define ENGINEER_WRIST_SUBSYSTEM_HPP_

#include <aruwlib/control/subsystem.hpp>
#include <aruwlib/motor/dji_motor.hpp>

#include "aruwsrc/algorithms/turret_pid.hpp"

#include "util_macros.hpp"

namespace aruwsrc
{
namespace engineer
{
class EngineerWristSubsystem : public aruwlib::control::Subsystem
{
public:
    static constexpr aruwlib::motor::MotorId LEFT_MOTOR_ID = aruwlib::motor::MOTOR1;
    static constexpr aruwlib::motor::MotorId RIGHT_MOTOR_ID = aruwlib::motor::MOTOR2;
    static constexpr aruwlib::can::CanBus CAN_BUS_MOTORS = aruwlib::can::CanBus::CAN_BUS1;
    static constexpr float MIN_WRIST_ANGLE = 0.0f;
    static constexpr float MAX_WRIST_ANGLE = 180.0f;

    explicit EngineerWristSubsystem(aruwlib::Drivers *drivers);
    ~EngineerWristSubsystem() = default;

    void refresh() override;

    mockable bool wristCalibrateHere();

    mockable void setWristAngle(float newAngle);

    mockable_inline float getWristAngle() const;

    mockable_inline float getWristDesiredAngle() const { return desiredWristAngle; }

private:
    static constexpr float WRIST_GEAR_RATIO = 19.0f;

    // PID values
    static constexpr float PID_P = 80000.0f;
    static constexpr float PID_I = 0.0f;
    static constexpr float PID_D = 800000.0f;
    static constexpr float PID_MAX_ERROR_SUM = 0.0f;
    static constexpr float PID_MAX_OUTPUT = 16000;

    aruwlib::motor::DjiMotor leftMotor;
    aruwlib::motor::DjiMotor rightMotor;

    aruwsrc::algorithms::TurretPid leftPositionPid;
    aruwsrc::algorithms::TurretPid rightPositionPid;

    // Desired angle in radian, unwrapped
    float desiredWristAngle;

    // Angle the wrist is initially calibrated to as a zero reference point
    float wristCalibratedAngleLeft;
    float wristCalibratedAngleRight;

    // If the wrist has been calibrated or not
    bool wristIsCalibrated;

    void wristRunPositionPid();

    inline float getUncalibratedWristAngleLeft() const;
    inline float getUncalibratedWristAngleRight() const;
};
}  // namespace engineer
}  // namespace aruwsrc

#endif  //  ENGINEER_WRIST_SUBSYSTEM_HPP_
