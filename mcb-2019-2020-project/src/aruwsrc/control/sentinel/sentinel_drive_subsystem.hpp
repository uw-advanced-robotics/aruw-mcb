#ifndef __SUBSYSTEM_SENTINEL_DRIVE_HPP__
#define __SUBSYSTEM_SENTINEL_DRIVE_HPP__

#include <modm/math/filter/pid.hpp>
#include "src/aruwlib/control/command_scheduler.hpp"
#include "src/aruwlib/control/subsystem.hpp"
#include "src/aruwlib/motor/dji_motor.hpp"

using namespace aruwlib::control;

namespace aruwsrc
{

namespace control
{

class SentinelDriveSubsystem : public Subsystem
{
 public:
    SentinelDriveSubsystem(
        aruwlib::motor::MotorId leftMotorId = LEFT_MOTOR_ID,
        aruwlib::motor::MotorId rightMotorId = RIGHT_MOTOR_ID)
        : leftWheel(leftMotorId, CAN_BUS_MOTORS, false, "left sentinel drive motor"),
        rightWheel(rightMotorId, CAN_BUS_MOTORS, false, "right sentinel drive motor"),
        velocityPidLeftWheel(PID_P, PID_I, PID_D, PID_MAX_ERROR_SUM, PID_MAX_OUTPUT),
        velocityPidRightWheel(PID_P, PID_I, PID_D, PID_MAX_ERROR_SUM, PID_MAX_OUTPUT),
        desiredRpm(0)
    {}

    float absolutePosition(void);

    void setDesiredRpm(float desRpm);

    void refresh(void);

    static constexpr float RAIL_LENGTH = 4650;

 private:
    static const aruwlib::motor::MotorId LEFT_MOTOR_ID;
    static const aruwlib::motor::MotorId RIGHT_MOTOR_ID;
    const aruwlib::can::CanBus CAN_BUS_MOTORS = aruwlib::can::CanBus::CAN_BUS1;
    using leftLimitSwitch = Board::DigitalInPinA;
    using rightLimitSwitch = Board::DigitalInPinB;

    const float PID_P = 5.0f;
    const float PID_I = 0.0f;
    const float PID_D = 0.1f;
    const float PID_MAX_ERROR_SUM = 0.0f;
    const float PID_MAX_OUTPUT = 16000;

    static constexpr float WHEEL_RADIUS = 35;
    static constexpr float GEAR_RATIO = 19.0f;

    aruwlib::motor::DjiMotor leftWheel;

    aruwlib::motor::DjiMotor rightWheel;

    modm::Pid<float> velocityPidLeftWheel;

    modm::Pid<float> velocityPidRightWheel;

    float desiredRpm;
    float leftZeroRailOffset = 0;
    float rightZeroRailOffset = 0;

    void resetOffsetFromLimitSwitch(void);

    float distanceFromEncoder(aruwlib::motor::DjiMotor* motor);

    void updateMotorRpmPid(
        modm::Pid<float>* pid,
        aruwlib::motor::DjiMotor* const motor,
        float desiredRpm
    );
};

}  // namespace control

}  // namespace aruwsrc

#endif
