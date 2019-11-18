#ifndef __AGITATOR_SUBSYSTEM_HPP__
#define __AGITATOR_SUBSYSTEM_HPP__

#include <modm/math/filter/pid.hpp>
#include "src/control/subsystem.hpp"
#include "src/motor/dji_motor.hpp"

namespace aruwsrc
{

namespace control
{

class AgitatorSubsystem : aruwlib::control::Subsystem {
 public:
    AgitatorSubsystem(uint16_t gearRatio);

    void refresh(void);

    void setAgitatorAngle(float newAngle);

 private:
    const float PID_P = 10.0f;
    const float PID_I = 0.0f;
    const float PID_D = 0.0f;
    const float PID_MAX_ERR_SUM = 0.0f;
    const float PID_MAX_OUT = 10000.0f;

    const aruwlib::motor::MotorId AGITATOR_MOTOR_ID = aruwlib::motor::MOTOR1;
    const aruwlib::can::CanBus AGITATOR_MOTOR_CAN_BUS = aruwlib::can::CanBus::CAN_BUS1;

    modm::Pid<float> agitatorPositionPid;

    aruwlib::motor::DjiMotor agitatorMotor;

    uint16_t agitatorGearRatio;

    float desiredAgitatorAngle;

    float agitatorEncoderToPosition(void);

    void agitatorRunPositionPid(void);
};

}  // namespace control

}  // namespace aruwsrc

#endif
