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

#include <modm/math/filter/pid.hpp>
#include "src/control/subsystem.hpp"
#include "src/motor/dji_motor.hpp"
#include "src/control/example-command.hpp"

using namespace aruwlib::control;

namespace aruwsrc
{

namespace control
{

class ExampleSubsystem : public Subsystem
{
 public:
    ExampleSubsystem(
        float p,
        float i,
        float d,
        float maxErrorSum,
        float maxOut,
        aruwlib::motor::MotorId leftMotorId,
        aruwlib::motor::MotorId rightMotorId
    );

    ~ExampleSubsystem()
    {
        delete[] m1;
        delete[] m2;
    }

    void setDesiredRpm(float desRpm);

    void refresh(void);

 private:
    aruwlib::motor::DjiMotor* m1;

    aruwlib::motor::DjiMotor* m2;

    float desiredRpm;

    modm::SmartPointer velocityPidLeftWheel;

    modm::SmartPointer velocityPidRightWheel;

    void updateMotorRpmPid(
        modm::Pid<float>* pid,
        aruwlib::motor::DjiMotor* motor,
        float desiredRpm
    );

    modm::Pid<float>* getPidPointer(modm::SmartPointer smrtPtr);

    aruwlib::motor::DjiMotor* getMotorPointer(modm::SmartPointer smrtPtr);
};

}  // namespace control

}  // namespace aruwsrc

#endif
