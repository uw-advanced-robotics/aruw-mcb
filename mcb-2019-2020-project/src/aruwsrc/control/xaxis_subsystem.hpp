#ifndef __SUBSYSTEM_XAXIS_HPP__
#define __SUBSYSTEM_XAXIS_hpp__

#include <modm/math/filter/pid.hpp>
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

     XAxisSubsystem(
        aruwlib::motor::MotorId xAxisId = XAXIS_MOTOR_ID)
        : XAxisMotor(xAxisId, CAN_BUS_MOTORS, true)
        xAxisVelocity(PID_P, PID_I, PID_D, PID_MAX_ERROR_SUM, PID_MAX_OUTPUT),
        setDesiredRpm(0.0)
    {}

    void setDesiredRpm(float rpm); 

    void refresh(void); 

    void setDisplacement(); 


 private: 
    static const aruwlib::motor::MotorId XAXIS_MOTOR_ID; 
    aruwlib::motor::DjiMotor XAxisMotor; 

    const aruwlib::can::CanBus CAN_BUS_MOTORS = aruwlib::can::CanBus::CAN_BUS1;

    modm::Pid<float> xAxisVelocity; 
    float xaxisRpm; // set value later


}; 

}

}
#endif

