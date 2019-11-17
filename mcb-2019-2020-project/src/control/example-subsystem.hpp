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
    ExampleSubsystem();

    void setDesiredRpm(float desRpm);

    void refresh(void);

 private:
    const float PID_P = 10.0f;
    const float PID_I = 0.0f;
    const float PID_D = 0.0f;
    const float PID_MAX_ERROR_SUM = 0.0f;
    const float PID_MAX_OUTPUT = 16000;

    const aruwlib::motor::MotorId LEFT_MOTOR_ID = aruwlib::motor::MOTOR4;
    const aruwlib::motor::MotorId RIGHT_MOTOR_ID = aruwlib::motor::MOTOR5;
    const aruwlib::can::CanBus CAN_BUS_MOTORS = aruwlib::can::CanBus::CAN_BUS1;

    aruwlib::motor::DjiMotor leftWheel;

    aruwlib::motor::DjiMotor rightWheel;

    modm::Pid<float> velocityPidLeftWheel;

    modm::Pid<float> velocityPidRightWheel;

    float desiredRpm;

    void updateMotorRpmPid(
        modm::Pid<float>& pid,
        aruwlib::motor::DjiMotor& motor,
        float desiredRpm
    );
};

}  // namespace control

}  // namespace aruwsrc

#endif
