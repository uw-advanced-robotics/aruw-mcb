#ifndef XAXIS_SUBSYSTEM_HPP_
#define XAXIS_SUBSYSTEM_HPP_

#include <modm/math/filter/pid.hpp>
#include <modm/math/filter/ramp.hpp>
#include "src/aruwlib/control/subsystem.hpp"
#include "src/aruwlib/motor/dji_motor.hpp"

namespace aruwsrc
{
namespace engineer
{
class XAxisSubsystem : public aruwlib::control::Subsystem {

public:
     XAxisSubsystem(aruwlib::motor::MotorId yAxisId = XAXIS_MOTOR_ID)
        : yAxisMotor(yAxisId, CAN_BUS_MOTORS, true),
        yAxisPositionPid(PID_P, PID_I, PID_D, PID_MAX_ERROR_SUM, PID_MAX_OUTPUT),
        yAxisRamp(0.1f, 0.1f, currentPosition)
    {}

    typedef enum {
       MIN_DISTANCE,
       CENTER_DISTANCE,
       MAX_DISTANCE,
    } Position;

    void setPosition(Position p);

    void initializeYAxis();

    void refresh();

private:
    static constexpr aruwlib::motor::MotorId XAXIS_MOTOR_ID = aruwlib::motor::MOTOR8;
    const aruwlib::can::CanBus CAN_BUS_MOTORS = aruwlib::can::CanBus::CAN_BUS1;

    Position yAxisPosition = MIN_DISTANCE; 

    static constexpr float PID_P = 100000.0f;
    static constexpr float PID_I = 0.0f;
    static constexpr float PID_D = 1000000.0f;
    static constexpr float PID_MAX_ERROR_SUM = 0.0f;
    static constexpr float PID_MAX_OUTPUT = 16000.0f;

    // units below are in centimeter (cm)
    static constexpr float MIN_DIST = 0.0f;
    static constexpr float CENTER_DIST = 15.0f; 
    static constexpr float MAX_DIST = 30.0f;
    static constexpr float Y_AXIS_PULLEY_RADIUS = 2.5f;
    static const int GM_3510_GEAR_RATIO = 19;

    int64_t startEncoder = 0;
    bool isInitialized = false; 

    aruwlib::motor::DjiMotor yAxisMotor;
    modm::Pid<float> yAxisPositionPid;
    modm::filter::Ramp<float> yAxisRamp;

    float currentPosition = 15.0f;
    float desiredPosition;

    void updateMotorDisplacement(
        aruwlib::motor::DjiMotor* const motor,
        modm::filter::Ramp<float>* ramp
    );

    float getPosition() const;
};  // class XAxisSubsystem
}  // namespace engineer
}  // namespace control
#endif  // XAXIS_SUBSYSTEM_HPP_
