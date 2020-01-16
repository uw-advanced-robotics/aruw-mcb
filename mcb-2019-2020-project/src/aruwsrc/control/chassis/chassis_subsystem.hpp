#ifndef __CHASSIS_SUBSYSTEM_HPP__
#define __CHASSIS_SUBSYSTEM_HPP__

#include <modm/math/filter/pid.hpp>
#include "src/aruwlib/control/subsystem.hpp"
#include "src/aruwlib/motor/dji_motor.hpp"
#include "src/aruwlib/algorithms/kalman.hpp"

using namespace aruwlib::control;

namespace aruwsrc
{

namespace chassis
{

/**
 * terminology:
 *     - anywhere 'x' is used, we mean this to be when looking down
 *       at the robot, the vertical axis, and likewise, 'y' is the
 *       horizontal axis of the robot
 *     - in other words, 'x' is the bow/stern and 'y' is starboard/
 *       port in boat terms
 */ 
class ChassisSubsystem : public Subsystem {
 public:
    // public constants
    static const int MAX_WHEEL_SPEED_SINGLE_MOTOR = 9000;

 private:
    #if defined(TARGET_SOLDIER)
    // velocity pid gains and constants
    const float VELOCITY_PID_KP            = 10.0f;
    const float VELOCITY_PID_KI            = 0.0f;
    const float VELOCITY_PID_KD            = 0.0f;
    const float VELOCITY_PID_MAX_ERROR_SUM = 0.0f;
    const float VELOCITY_PID_MAX_OUTPUT    = 16000.0f;

    // rotation pid gains and constants
    // no i, max error sum the same as MAX_WHEEL_SPEED_SINGLE_MOTOR, proportional
    // gain specified by user
    static constexpr double CHASSIS_REVOLVE_PID_MAX_P = MAX_WHEEL_SPEED_SINGLE_MOTOR;
    // derivative term used in chassis pid
    static constexpr float CHASSIS_REVOLVE_PID_KD     = 235.f;
    // the maximum revolve error before we start using the derivative term
    static const int MIN_ERROR_ROTATION_D                = 35;

    // mechanical chassis constants, all in mm
    // radius of the wheels
    static constexpr float WHEEL_RADIUS          = 76.0f;
    // distance from center of the two front wheels
    static constexpr float WHEELTRACK            = 366.0f;
    // distance from center of the front and rear wheels
    static constexpr float WHEELBASE             = 366.0f;
    // gimbal offset from the center of the chassis, see note above for explanation of x and y
    static constexpr float GIMBAL_X_OFFSET       = 0.0f;
    static constexpr float GIMBAL_Y_OFFSET       = 0.0f;
    static constexpr float CHASSIS_GEARBOX_RATIO = (1.0f / 19.0f);

    #elif defined(TARGET_HERO)
    // velocity pid gains and constants
    const float VELOCITY_PID_KP            = 0.0f;
    const float VELOCITY_PID_KI            = 0.0f;
    const float VELOCITY_PID_KD            = 0.0f;
    const float VELOCITY_PID_MAX_ERROR_SUM = 0.0f;
    const float VELOCITY_PID_MAX_OUTPUT    = 16000.0f;

    // rotation pid gains and constants
    // no i, max error sum is MAX_WHEEL_SPEED_SINGLE_MOTOR, proportional gain specified by user
    static constexpr double CHASSIS_REVOLVE_PID_MAX_P = 0.0;
    static constexpr float CHASSIS_REVOLVE_PID_KD     = 0.0;
    static const int MIN_ERROR_ROTATION_D                = 0;

    // mechanical chassis constants
    static constexpr float WHEEL_RADIUS          = 76.0f;
    static constexpr float WHEELTRACK            = 517.0f;
    static constexpr float WHEELBASE             = 600.0f;
    static constexpr float GIMBAL_X_OFFSET       = 175.0f;
    static constexpr float GIMBAL_Y_OFFSET       = 0.0f;
    static constexpr float CHASSIS_GEARBOX_RATIO = (1.0f / 19.0f);

    #else
    // velocity pid gains and constants
    const float VELOCITY_PID_KP            = 0.0f;
    const float VELOCITY_PID_KI            = 0.0f;
    const float VELOCITY_PID_KD            = 0.0f;
    const float VELOCITY_PID_MAX_ERROR_SUM = 0.0f;
    const float VELOCITY_PID_MAX_OUTPUT    = 16000.0f;

    // rotation pid gains and constants
    // no i, max error sum is MAX_WHEEL_SPEED_SINGLE_MOTOR, proportional gain specified by user
    static constexpr double CHASSIS_REVOLVE_PID_MAX_P = 0.0;
    static constexpr float CHASSIS_REVOLVE_PID_KD     = 0.0;
    static const int MIN_ERROR_ROTATION_D                = 0;

    // mechanical chassis constants
    static constexpr float WHEEL_RADIUS          = 76.0f;
    static constexpr float WHEELTRACK            = 517.0f;
    static constexpr float WHEELBASE             = 600.0f;
    static constexpr float GIMBAL_X_OFFSET       = 175.0f;
    static constexpr float GIMBAL_Y_OFFSET       = 0.0f;
    static constexpr float CHASSIS_GEARBOX_RATIO = (1.0f / 19.0f);
    #endif

    // hardware constants, not specific to any particular chassis
    static constexpr aruwlib::motor::MotorId LEFT_FRONT_MOTOR_ID  = aruwlib::motor::MOTOR2;
    static constexpr aruwlib::motor::MotorId LEFT_BACK_MOTOR_ID   = aruwlib::motor::MOTOR3;
    static constexpr aruwlib::motor::MotorId RIGHT_FRONT_MOTOR_ID = aruwlib::motor::MOTOR1;
    static constexpr aruwlib::motor::MotorId RIGHT_BACK_MOTOR_ID  = aruwlib::motor::MOTOR4;
    static constexpr aruwlib::can::CanBus CAN_BUS_MOTORS = aruwlib::can::CanBus::CAN_BUS1;

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
        leftFrontMotor(leftFrontMotorId, CAN_BUS_MOTORS, false),
        leftBackMotor(leftBackMotorId, CAN_BUS_MOTORS, false),
        rightFrontMotor(rightFrontMotorId, CAN_BUS_MOTORS, false),
        rightBackMotor(rightBackMotorId, CAN_BUS_MOTORS, false),
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

    void setDesiredOutput(float x, float y, float r);

    /**
     * run chassis rotation pid on some actual turret angle offset
     * 
     * @param currentAngleError the error as an angle. For autorotation,
     * error between gimbal and center of chassis.
     * 
     * @param kp proportional gain for pid caluclation
     * 
     * @retval a desired rotation speed (wheel speed)
     */
    float chassisSpeedZPID(float currentAngleError, float kp);

    void refresh() override;

    // Returns the value used for chassis movement forward and backward, between -1 and 1
    static float getChassisX();

    // Returns the value used for chassis movement side to side, between -1 and 1
    static float getChassisY();

    // Returns the value used for chassis rotation, between -1 and 1
    static float getChassisR();

 private:
    /**
     * When you input desired x, y, an r values, this function translates
     * and sets the rpm of individual chassis motors
     */
    void mecanumDriveCalculate(float x, float y, float r, float maxWheelSpeed);

    void updateMotorRpmPid(
        modm::Pid<float>* pid,
        aruwlib::motor::DjiMotor* const motor,
        float desiredRpm
    );
};

}  // namespace chassis

}  // namespace aruwsrc

#endif
