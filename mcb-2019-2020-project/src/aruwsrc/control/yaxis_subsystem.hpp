#ifndef __SUBSYSTEM_YAXIS_HPP__
#define __SUBSYSTEM_YAXIS_HPP__

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

class YAxisSubsystem : public Subsystem {

 public:
     YAxisSubsystem(aruwlib::motor::MotorId yAxisId = YAXIS_MOTOR_ID)
        : yAxisMotor(yAxisId, CAN_BUS_MOTORS, true),
        yAxisPositionPid(PID_P, PID_I, PID_D, PID_MAX_ERROR_SUM, PID_MAX_OUTPUT),
        yAxisRamp(0.1f, 0.1f, currentPosition)
    {}

    typedef enum {
       MIN_DISTANCE,
       CENTER_DISTANCE,
       MAX_DISTANCE,
    } Position ;  

    void setPosition(Position p);

    void refresh(void);

 private:
    static constexpr aruwlib::motor::MotorId YAXIS_MOTOR_ID = aruwlib::motor::MOTOR8;
    const aruwlib::can::CanBus CAN_BUS_MOTORS = aruwlib::can::CanBus::CAN_BUS1;

    Position yAxisPosition = MIN_DISTANCE; 

    const float PID_P = 100.0f;
    const float PID_I = 0.0f;
    const float PID_D = 0.0f;
    const float PID_MAX_ERROR_SUM = 0.0f;
    const float PID_MAX_OUTPUT = 16000;

    // units below are in centimeter (cm)
    const float MIN_DIST = 0.0f;
    const float CENTER_DIST = 15.0f; 
    const float MAX_DIST = 30.0f;
    const float Y_AXIS_PULLEY_RADIUS = 2.5f;
    const int GM_3510_GEAR_RATIO = 19;

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
};

}

}
#endif