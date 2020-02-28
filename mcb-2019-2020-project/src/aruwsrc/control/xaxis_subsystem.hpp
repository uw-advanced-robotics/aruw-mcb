#ifndef __SUBSYSTEM_XAXIS_HPP__
#define __SUBSYSTEM_XAXIS_HPP__

#include <modm/math/filter/pid.hpp>
#include <modm/math/filter/ramp.hpp>
#include "src/aruwlib/control/command_scheduler.hpp"
#include "src/aruwlib/control/subsystem.hpp"
#include "src/aruwlib/motor/dji_motor.hpp"

using namespace aruwlib::control;

namespace aruwsrc
{

namespace control
{

class XAxisSubsystem : public Subsystem {

 public:
     XAxisSubsystem(aruwlib::motor::MotorId xAxisId = XAXIS_MOTOR_ID)
        : xAxisMotor(xAxisId, CAN_BUS_MOTORS, true),
        xAxisPositionPid(PID_P, PID_I, PID_D, PID_MAX_ERROR_SUM, PID_MAX_OUTPUT),
        xAxisRamp(0.1f, 0.1f, currentPosition)
    {}

    enum Position {
       MIN_DISTANCE,
       CENTER_DISTANCE,
       MAX_DISTANCE,
    };  

    void setPosition(Position p);

    void refresh(void);

 private:
    static constexpr aruwlib::motor::MotorId XAXIS_MOTOR_ID = aruwlib::motor::MOTOR8;
    const aruwlib::can::CanBus CAN_BUS_MOTORS = aruwlib::can::CanBus::CAN_BUS1;

    Position xAxisPosition = MIN_DISTANCE; 

    const float PID_P = 100.0f;
    const float PID_I = 0.0f;
    const float PID_D = 0.0f;
    const float PID_MAX_ERROR_SUM = 0.0f;
    const float PID_MAX_OUTPUT = 16000;

    // units below are in centimeter (cm)
    const float MIN_DISTANCE = 0.0f;
    const float CENTER_DISTANCE = 15.0f; 
    const float MAX_DISTANCE = 30.0f;
    const float X_AXIS_PULLEY_RADIUS = 2.5f;
    const int GM_3510_GEAR_RATIO = 19;

    aruwlib::motor::DjiMotor xAxisMotor;
    modm::Pid<float> xAxisPositionPid;
    modm::filter::Ramp<float> xAxisRamp;

    float currentPosition = 15.0f;
    float desiredPosition;

    void updateMotorDisplacement(
        modm::Pid<float>* pid,
        aruwlib::motor::DjiMotor* const motor,
        modm::filter::Ramp<float>* ramp
    );

    float XAxisSubsystem::getPosition() const;
};

}

}
#endif