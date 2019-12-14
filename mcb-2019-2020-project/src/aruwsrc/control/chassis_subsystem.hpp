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
    static const float WHEELBASE_RADIUS = 76.0f;
    static const float PERIMETER = 500.0f;
    static const float WHEELTRACK = 360.0f;
    static const float WHEELBASE = 300.0f;
    static const float GIMBAL_X_OFFSET = 0.0f;
    static const float GIMBAL_Y_OFFSET = 0.0f;
    static const float CHASSIS_GEARBOX_RATIO = (1.0f / 19.0f);
    static const float RADIAN_COEF = 57.3f;

    static const int OMNI_SPEED_MAX = 9000;
    static const int CHAS_CURRENT_LIMIT = OMNI_SPEED_MAX * 4;
    static const int STANDARD_MAX_NORMAL = 9000;
    static const int REVOLVE_MAX_NORMAL = 9000;
    static const float REVOLVE_KD = 235.f;
    static const int REVOLVE_ANGLE = 35;

    static const float WARNING_REMAIN_POWER = 60;


    // chassis proportional gains (what we multiply the remote and key values by)
    static const float KRC_MECH_CHASSIS_STANDARD = 14.0f;  // no auto rotate, x and y
    static const float KRC_MECH_CHASSIS_REVOLVE  = 11.4f;  // no auto rotate, z
    static const float KRC_GYRO_CHASSIS_STANDARD = 14.0f;  // when auto rotating
    static const float KRC_GYRO_CHASSIS_REVOLVE  = -10.0f;

    static const float KKEY_MECH_CHASSIS_REVOLVE = 40.0f;
    static const float KKEY_MECH_CHASSIS_REVOLVE = -10.0f;

    ChassisSubsystem(
        aruwlib::motor::MotorId leftTopMotorId = LEFT_FRONT_MOTOR_ID,
        aruwlib::motor::MotorId leftBotMotorId = LEFT_BACK_MOTOR_ID,
        aruwlib::motor::MotorId rightTopMotorId = RIGHT_FRONT_MOTOR_ID,
        aruwlib::motor::MotorId rightBotMotorId = RIGHT_BACK_MOTOR_ID
    ):
        leftTopMotor(leftTopMotorId, CAN_BUS_MOTORS),
        leftBotMotor(leftBotMotorId, CAN_BUS_MOTORS),
        rightTopMotor(rightTopMotorId, CAN_BUS_MOTORS),
        rightBotMotor(rightBotMotorId, CAN_BUS_MOTORS),
        leftTopVelocityPid(PID_P, PID_I, PID_D, PID_MAX_ERROR_SUM, PID_MAX_OUTPUT),
        leftBotVelocityPid(PID_P, PID_I, PID_D, PID_MAX_ERROR_SUM, PID_MAX_OUTPUT),
        rightTopVelocityPid(PID_P, PID_I, PID_D, PID_MAX_ERROR_SUM, PID_MAX_OUTPUT),
        rightBotVelocityPid(PID_P, PID_I, PID_D, PID_MAX_ERROR_SUM, PID_MAX_OUTPUT),
        leftFrontRpm(0),
        leftBackRpm(0),
        rightFrontRpm(0),
        rightBackRpm(0)
    {
        KalmanCreate(&chassisErrorKalman, 1.0f, 0.0f);
    }

    void setDesiredOutput(float x, float y, float r);

    /**
     * run chassis rotation pid on some gimbal encoder tick offset
     * 
     * @param errorReal the error in encoder ticks. For autorotation,
     * error between gimbal and center of chassis.
     * 
     * @param kp proportional gain for pid caluclation
     */
    float chassisSpeedZPID(int16_t errorReal, float kp);

    void refresh(void);

private:
    const float PID_P = 10.0f;
    const float PID_I = 0.0f;
    const float PID_D = 0.0f;
    const float PID_MAX_ERROR_SUM = 0.0f;
    const float PID_MAX_OUTPUT = 16000.0f;

    static const aruwlib::motor::MotorId LEFT_FRONT_MOTOR_ID;
    static const aruwlib::motor::MotorId LEFT_BACK_MOTOR_ID;
    static const aruwlib::motor::MotorId RIGHT_FRONT_MOTOR_ID;
    static const aruwlib::motor::MotorId RIGHT_BACK_MOTOR_ID;
    const aruwlib::can::CanBus CAN_BUS_MOTORS = aruwlib::can::CanBus::CAN_BUS1;

    aruwlib::motor::DjiMotor leftTopMotor;
    aruwlib::motor::DjiMotor leftBotMotor;
    aruwlib::motor::DjiMotor rightTopMotor;
    aruwlib::motor::DjiMotor rightBotMotor;

    modm::Pid<float> leftTopVelocityPid;
    modm::Pid<float> leftBotVelocityPid;
    modm::Pid<float> rightTopVelocityPid;
    modm::Pid<float> rightBotVelocityPid;

    float leftFrontRpm;
    float leftBackRpm;
    float rightFrontRpm;
    float rightBackRpm;

    // rotation variables
    modm::Pid<float> chassisSpeedZPid;

    extKalman_t chassisErrorKalman; // make sure to initialize this

    float speed_z_pterm = 0;
    float speed_z_iterm = 0;
    float speed_z_dterm = 0;

    float Chassis_Revolve_Move_Max;

    void chassisOmniMoveCalculate(float x, float y, float z);

    void chassisPowerLimit(void);

    void updateMotorRpmPid(
        modm::Pid<float>* pid,
        aruwlib::motor::DjiMotor* const motor,
        float desiredRpm
    );
};

}  // namespace control

}  // namespace aruwsrc

#endif