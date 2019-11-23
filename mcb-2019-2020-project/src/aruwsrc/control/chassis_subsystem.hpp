#ifndef __CHASSIS_SUBSYSTEM_HPP__
#define __CHASSIS_SUBSYSTEM_HPP__

#include <modm/math/filter/pid.hpp>
#include "src/control/subsystem.hpp"
#include "src/motor/dji_motor.hpp"

using namespace aruwlib::control;

namespace aruwsrc
{

namespace control
{

class ChassisSubsystem : public Subsystem {
public:
    ChassisSubsystem(
        aruwlib::motor::MotorId leftTopMotorId = LEFT_TOP_MOTOR_ID,
        aruwlib::motor::MotorId leftBotMotorId = LEFT_BOT_MOTOR_ID,
        aruwlib::motor::MotorId rightTopMotorId = RIGHT_TOP_MOTOR_ID,
        aruwlib::motor::MotorId rightBotMotorId = RIGHT_BOT_MOTOR_ID
    ):
        leftTopMotor(leftTopMotorId, CAN_BUS_MOTORS),
        leftBotMotor(leftBotMotorId, CAN_BUS_MOTORS),
        rightTopMotor(rightTopMotorId, CAN_BUS_MOTORS),
        rightBotMotor(rightBotMotorId, CAN_BUS_MOTORS),
        leftTopVelocityPid(PID_P, PID_I, PID_D, PID_MAX_ERROR_SUM, PID_MAX_OUTPUT),
        leftBotVelocityPid(PID_P, PID_I, PID_D, PID_MAX_ERROR_SUM, PID_MAX_OUTPUT),
        rightTopVelocityPid(PID_P, PID_I, PID_D, PID_MAX_ERROR_SUM, PID_MAX_OUTPUT),
        rightBotVelocityPid(PID_P, PID_I, PID_D, PID_MAX_ERROR_SUM, PID_MAX_OUTPUT),
        leftTopRpm(0),
        leftBotRpm(0),
        rightTopRpm(0),
        rightBotRpm(0)
    {}

  void setDesiredOutput(float x, float y, float r);

  void refresh(void);

private:
    const float PID_P = 10.0f;
    const float PID_I = 0.0f;
    const float PID_D = 0.0f;
    const float PID_MAX_ERROR_SUM = 0.0f;
    const float PID_MAX_OUTPUT = 16000.0f;

    static const aruwlib::motor::MotorId LEFT_TOP_MOTOR_ID;
    static const aruwlib::motor::MotorId LEFT_BOT_MOTOR_ID;
    static const aruwlib::motor::MotorId RIGHT_TOP_MOTOR_ID;
    static const aruwlib::motor::MotorId RIGHT_BOT_MOTOR_ID;
    const aruwlib::can::CanBus CAN_BUS_MOTORS = aruwlib::can::CanBus::CAN_BUS1;

    aruwlib::motor::DjiMotor leftTopMotor;
    aruwlib::motor::DjiMotor leftBotMotor;
    aruwlib::motor::DjiMotor rightTopMotor;
    aruwlib::motor::DjiMotor rightBotMotor;

    modm::Pid<float> leftTopVelocityPid;
    modm::Pid<float> leftBotVelocityPid;
    modm::Pid<float> rightTopVelocityPid;
    modm::Pid<float> rightBotVelocityPid;

    float leftTopRpm;
    float leftBotRpm;
    float rightTopRpm;
    float rightBotRpm;

    void updateMotorRpmPid(
        modm::Pid<float>* pid,
        aruwlib::motor::DjiMotor* const motor,
        float desiredRpm
    );
};

}  // namespace control

}  // namespace aruwsrc

#endif