#ifndef __CHASSIS_SUBSYSTEM_HPP__
#define __CHASSIS_SUBSYSTEM_HPP__

#include <modm/math/filter/pid.hpp>
#include "src/aruwlib/control/subsystem.hpp"
#include "src/aruwlib/motor/dji_motor.hpp"
#include "src/aruwlib/algorithms/kalman.h"

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
    {
        KalmanCreate(&chassisErrorKalman, 1.0f, 0.0f);
    }

    void setDesiredOutput(float x, float y, float r);

    float Chassis_SpeedZ_PID(int16_t ErrorReal, float kp);

    void refresh(void);

private:
    const int Omni_Speed_Max = 9000;//7600
    const int STANDARD_MAX_NORMAL = 9000;//7600
    const int REVOLVE_MAX_NORMAL = 9000;//7600
    const float REVOLVE_KD = 235.f;
    const int REVOLVE_ANGLE = 35;

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

    // rotation variables
    modm::Pid<float> chassisSpeedZPid;

    extKalman_t chassisErrorKalman; // make sure to initialize this

    float speed_z_pterm = 0;
    float speed_z_iterm = 0;
    float speed_z_dterm = 0;

    float Chassis_Revolve_Move_Max;

    void updateMotorRpmPid(
        modm::Pid<float>* pid,
        aruwlib::motor::DjiMotor* const motor,
        float desiredRpm
    );
};

}  // namespace control

}  // namespace aruwsrc

#endif