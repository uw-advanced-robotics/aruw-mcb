/**
 * This is part of aruw's library.
 * 
 * This is example code for running friction wheels. As you can
 * see, there is a generic update pid loop that is independent of
 * what command is given to the subsystem. Additionally, the
 * subsystem contains variables specific to the subsystem
 * (pid controllers, motors, etc). If a control loop is specific
 * to a command, it should NOT be in a subsystem. For example,
 * control code to pulse the friction wheels should be located
 * outside of this class because pulsing is a specific command.
 */

#ifndef __SUBSYSTEM_EXAMPLE_HPP__
#define __SUBSYSTEM_EXAMPLE_HPP__

#include "src/control/subsystem.hpp"
#include <modm/math/filter/pid.hpp>
#include "src/motor/dji_motor.hpp"
#include "src/control/command-example.hpp"

using namespace aruwlib::control;

namespace aruwsrc
{

namespace control
{

class SubsystemExample : public Subsystem
{
 public:
    SubsystemExample(
        float p,
        float i,
        float d,
        float maxOut,
        aruwlib::motor::MotorId leftMotorId,
        aruwlib::motor::MotorId rightMotorId
    ) {
        frictionWheelLeft = new aruwlib::motor::DjiMotor(
            leftMotorId, aruwlib::can::CanBus::CAN_BUS1);
        frictionWheelRight = new aruwlib::motor::DjiMotor(
            rightMotorId, aruwlib::can::CanBus::CAN_BUS1);
    
        velocityPidLeftWheel = new modm::Pid<float>(p, i, d, maxOut);
        velocityPidRightWheel = new modm::Pid<float>(p, i, d, maxOut);
    }

    void InitDefaultCommand(void);

    void setDesiredRpm(float desRpm);

    void refresh(void);

 private:
    float desiredRpm;

    aruwlib::motor::DjiMotor* frictionWheelLeft;

    aruwlib::motor::DjiMotor* frictionWheelRight;

    modm::Pid<float>* velocityPidLeftWheel;

    modm::Pid<float>* velocityPidRightWheel;
};

}  // namespace control

}  // namespace aruwsrc

#endif
