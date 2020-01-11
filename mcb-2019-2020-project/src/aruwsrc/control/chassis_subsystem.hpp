#ifndef __CHASSIS_SUBSYSTEM_HPP__
#define __CHASSIS_SUBSYSTEM_HPP__

#include <modm/math/filter/pid.hpp>
#include "src/aruwlib/control/subsystem.hpp"
#include "src/aruwlib/motor/dji_motor.hpp"
#include "src/aruwlib/algorithms/kalman.hpp"

using namespace aruwlib::control;

namespace aruwsrc
{

namespace control
{

class ChassisSubsystem : public Subsystem {
 public:
    // public constants
    static const int OMNI_SPEED_MAX = 9000;

 private:
    // velocity pid gains and constants
    const float VELOCITY_PID_KP            = 10.0f;
    const float VELOCITY_PID_KI            = 0.0f;
    const float VELOCITY_PID_KD            = 0.0f;
    const float VELOCITY_PID_MAX_ERROR_SUM = 0.0f;
    const float VELOCITY_PID_MAX_OUTPUT    = 16000.0f;

    // rotation pid gains and constants
    // no i, max error sum is OMNI_SPEED_MAX, proportional gain specified by user
    static constexpr double CHASSIS_REVOLVE_PID_MAX_P = 9000.0;
    static constexpr float CHASSIS_REVOLVE_PID_KD     = 235.f;
    static const int MAX_REVOLVE_ANGLE                = 35;

    // hardware constants
    static constexpr aruwlib::motor::MotorId LEFT_FRONT_MOTOR_ID  = aruwlib::motor::MOTOR2;
    static constexpr aruwlib::motor::MotorId LEFT_BACK_MOTOR_ID   = aruwlib::motor::MOTOR3;
    static constexpr aruwlib::motor::MotorId RIGHT_FRONT_MOTOR_ID = aruwlib::motor::MOTOR1;
    static constexpr aruwlib::motor::MotorId RIGHT_BACK_MOTOR_ID  = aruwlib::motor::MOTOR4;
    static constexpr aruwlib::can::CanBus CAN_BUS_MOTORS = aruwlib::can::CanBus::CAN_BUS1;

    // mechanical chassis constants
    static constexpr float WHEELBASE_RADIUS      = 76.0f;
    static constexpr float PERIMETER             = 500.0f;
    static constexpr float WHEELTRACK            = 360.0f;
    static constexpr float WHEELBASE             = 300.0f;
    static constexpr float GIMBAL_X_OFFSET       = 0.0f;
    static constexpr float GIMBAL_Y_OFFSET       = 0.0f;
    static constexpr float CHASSIS_GEARBOX_RATIO = (1.0f / 19.0f);

    // electrical power constants
    static const int CHAS_CURRENT_LIMIT         = OMNI_SPEED_MAX * 4;
    static constexpr float WARNING_REMAIN_POWER = 60;

    // motors
    aruwlib::motor::DjiMotor leftFrontMotor;
    aruwlib::motor::DjiMotor leftBackMotor;
    aruwlib::motor::DjiMotor rightFrontMotor;
    aruwlib::motor::DjiMotor rightBackMotor;

    // wheel velocity pid variables
    modm::Pid<float> leftFrontVelocityPid;
    modm::Pid<float> leftBackVelocityPid;
    modm::Pid<float> rightFrontVelocityPid;
    modm::Pid<float> rightBackVelocityPid;

    // translate all input into a desired wheel rpm when given
    float leftFrontRpm;
    float leftBackRpm;
    float rightFrontRpm;
    float rightBackRpm;

    // rotation pid variables
    ExtKalman chassisErrorKalman;
    float rotationPidP = 0;
    float rotationPidD = 0;

    // rotation pid parameters
    float errorPrevKalman = 0;
    float ErrorSum = 0;
    float errorPRotate = 0;
    float errorPRotateKalman = 0;

 public:
    ChassisSubsystem(
        aruwlib::motor::MotorId leftFrontMotorId = LEFT_FRONT_MOTOR_ID,
        aruwlib::motor::MotorId leftBackMotorId = LEFT_BACK_MOTOR_ID,
        aruwlib::motor::MotorId rightFrontMotorId = RIGHT_FRONT_MOTOR_ID,
        aruwlib::motor::MotorId rightBackMotorId = RIGHT_BACK_MOTOR_ID
    ):
        leftFrontMotor(leftFrontMotorId, CAN_BUS_MOTORS),
        leftBackMotor(leftBackMotorId, CAN_BUS_MOTORS),
        rightFrontMotor(rightFrontMotorId, CAN_BUS_MOTORS),
        rightBackMotor(rightBackMotorId, CAN_BUS_MOTORS),
        leftFrontVelocityPid(
            VELOCITY_PID_KP,
            VELOCITY_PID_KI,
            VELOCITY_PID_KD,
            VELOCITY_PID_MAX_ERROR_SUM,
            VELOCITY_PID_MAX_OUTPUT),
        leftBackVelocityPid(
            VELOCITY_PID_KP,
            VELOCITY_PID_KI,
            VELOCITY_PID_KD,
            VELOCITY_PID_MAX_ERROR_SUM,
            VELOCITY_PID_MAX_OUTPUT),
        rightFrontVelocityPid(
            VELOCITY_PID_KP,
            VELOCITY_PID_KI,
            VELOCITY_PID_KD,
            VELOCITY_PID_MAX_ERROR_SUM,
            VELOCITY_PID_MAX_OUTPUT),
        rightBackVelocityPid(
            VELOCITY_PID_KP,
            VELOCITY_PID_KI,
            VELOCITY_PID_KD,
            VELOCITY_PID_MAX_ERROR_SUM,
            VELOCITY_PID_MAX_OUTPUT),
        leftFrontRpm(0),
        leftBackRpm(0),
        rightFrontRpm(0),
        rightBackRpm(0)
    {
        KalmanCreate(&chassisErrorKalman, 1.0f, 0.0f);
    }

    void setDesiredOutput(float x, float y, float z);

    /**
     * run chassis rotation pid on some actual turret angle offset
     * 
     * @param errorReal the error as an angle. For autorotation,
     * error between gimbal and center of chassis.
     * 
     * @param kp proportional gain for pid caluclation
     * 
     * @retval a desired rotation speed (wheel speed)
     */
    float chassisSpeedZPID(float errorReal, float kp);

    void refresh(void);

 private:
    /**
     * When you input desired x, y, an z values, this function translates
     * and sets the rpm of individual chassis motors
     */
    void chassisOmniMoveCalculate(float x, float y, float z, float speedMax);

    void updateMotorRpmPid(
        modm::Pid<float>* pid,
        aruwlib::motor::DjiMotor* const motor,
        float desiredRpm
    );
};

}  // namespace control

}  // namespace aruwsrc

#endif
