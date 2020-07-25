#ifndef YAXIS_SUBSYSTEM_HPP_
#define YAXIS_SUBSYSTEM_HPP_

#include <aruwlib/control/subsystem.hpp>
#include <aruwlib/motor/dji_motor.hpp>
#include <modm/math/filter/ramp.hpp>
#include <aruwlib/algorithms/ramp.hpp>

#include "aruwsrc/algorithms/turret_pid.hpp"

namespace aruwsrc
{
namespace engineer
{
class YAxisSubsystem : public aruwlib::control::Subsystem
{
public:
    enum class Position
    {
        MIN_DISTANCE,
        CENTER_DISTANCE,
        MAX_DISTANCE,
    };

    YAxisSubsystem(aruwlib::motor::MotorId yAxisId = YAXIS_MOTOR_ID)
        : yAxisPosition(Position::MIN_DISTANCE),
          startEncoder(0),
          isInitialized(false),
          yAxisMotor(yAxisId, CAN_BUS_MOTORS, true, "yaxis motor"),
          yAxisPositionPid(
              PID_P,
              PID_I,
              PID_D,
              PID_MAX_ERROR_SUM,
              PID_MAX_OUTPUT,
              1.0f,
              0.0f,
              1.0f,
              0.0f),
          yAxisRamp(0.0f),
          currentPosition(0.0f),
          desiredPosition(0.0f),
          oldRampTime(0)
    {
    }

    void setPosition(Position p);

    void initializeYAxis();

    void refresh();

    float getCurrentPosition() const { return currentPosition; }
    float getDesiredPosition() const { return yAxisRamp.getTarget(); }

private:
    static constexpr aruwlib::motor::MotorId YAXIS_MOTOR_ID = aruwlib::motor::MOTOR8;
    static constexpr aruwlib::can::CanBus CAN_BUS_MOTORS = aruwlib::can::CanBus::CAN_BUS1;

    static constexpr float PID_P = 100000.0f;
    static constexpr float PID_I = 0.0f;
    static constexpr float PID_D = 1000000.0f;
    static constexpr float PID_MAX_ERROR_SUM = 0.0f;
    static constexpr float PID_MAX_OUTPUT = 16000.0f;
    static constexpr float RAMP_SPEED = 1.0f;

    // units below are in centimeter (cm)
    static constexpr float MIN_DIST = 0.0f;
    static constexpr float CENTER_DIST = 15.0f;
    static constexpr float MAX_DIST = 30.0f;
    static constexpr float Y_AXIS_PULLEY_RADIUS = 2.5f;

    // 19:1 gear ratio
    static constexpr float GM_3510_GEAR_RATIO = 19.0f;

    Position yAxisPosition;

    int64_t startEncoder;
    bool isInitialized;

    aruwlib::motor::DjiMotor yAxisMotor;
    algorithms::TurretPid yAxisPositionPid;
    aruwlib::algorithms::Ramp yAxisRamp;

    float currentPosition;
    float desiredPosition;
    uint32_t oldRampTime;

    void updateMotorDisplacement(
        aruwlib::motor::DjiMotor* const motor,
        modm::filter::Ramp<float>* ramp);

    float getPosition() const;
};  // class YAxisSubsystem
}  // namespace engineer
}  // namespace aruwsrc
#endif  // YAXIS_SUBSYSTEM_HPP_
