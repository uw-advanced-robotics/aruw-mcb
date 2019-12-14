#include "chassis_subsystem.hpp"
#include "src/aruwlib/algorithms/math_user_utils.hpp"

#define DEG_TO_RAD(val) (val * 3.14f / 180.0f)
#define PI (3.14f)

namespace aruwsrc
{

namespace control
{
    const aruwlib::motor::MotorId ChassisSubsystem::LEFT_FRONT_MOTOR_ID = aruwlib::motor::MOTOR4;
    const aruwlib::motor::MotorId ChassisSubsystem::LEFT_BACK_MOTOR_ID = aruwlib::motor::MOTOR5;
    const aruwlib::motor::MotorId ChassisSubsystem::RIGHT_FRONT_MOTOR_ID = aruwlib::motor::MOTOR6;
    const aruwlib::motor::MotorId ChassisSubsystem::RIGHT_BACK_MOTOR_ID = aruwlib::motor::MOTOR7;

    void ChassisSubsystem::setDesiredOutput(float x, float y, float r)
    {
        // TODO calculate motor rpm values based on given x, y, and r
        chassisOmniMoveCalculate(x, y, r);
    }

    void ChassisSubsystem::refresh()
    {
        updateMotorRpmPid(&leftTopVelocityPid, &leftTopMotor, leftFrontRpm);
        updateMotorRpmPid(&leftBotVelocityPid, &leftBotMotor, leftBackRpm);
        updateMotorRpmPid(&rightTopVelocityPid, &rightTopMotor, rightFrontRpm);
        updateMotorRpmPid(&rightBotVelocityPid, &rightBotMotor, rightBackRpm);
    }

    // todo fix all of this or insure it is correct
    void ChassisSubsystem::chassisOmniMoveCalculate(float x, float y, float z)
    {
        float rotateRatioFL, rotateRatioRF, rotateRatioBL, rotateRatioBR;
        float wheel_rpm_ratio = 60.0f / (PERIMETER * CHASSIS_GEARBOX_RATIO); // what is this
        float speed_max = OMNI_SPEED_MAX;
        float chassisRotationRatio = (WHEELBASE + WHEELTRACK) / 2.0f;
        float zTrans = z * RADIAN_COEF / chassisRotationRatio;

        rotateRatioFL = (chassisRotationRatio - GIMBAL_X_OFFSET - GIMBAL_Y_OFFSET) / RADIAN_COEF;
        rotateRatioRF = (chassisRotationRatio - GIMBAL_X_OFFSET + GIMBAL_Y_OFFSET) / RADIAN_COEF;
        rotateRatioBL = (chassisRotationRatio + GIMBAL_X_OFFSET - GIMBAL_Y_OFFSET) / RADIAN_COEF;
        rotateRatioBR = (chassisRotationRatio + GIMBAL_X_OFFSET + GIMBAL_Y_OFFSET) / RADIAN_COEF;
    
        leftFrontRpm  =  ( y + x + zTrans) / wheel_rpm_ratio * rotateRatioFL;
        rightFrontRpm = -(-y + x - zTrans) / wheel_rpm_ratio * rotateRatioRF;
        leftBackRpm   =  (-y + x + zTrans) / wheel_rpm_ratio * rotateRatioBL;
        rightBackRpm  = -( y + x - zTrans) / wheel_rpm_ratio * rotateRatioBR;

        leftFrontRpm  = aruwlib::algorithms::limitVal<float> (leftFrontRpm,  -speed_max, speed_max);
        rightFrontRpm = aruwlib::algorithms::limitVal<float> (rightFrontRpm, -speed_max, speed_max);
        leftBackRpm   = aruwlib::algorithms::limitVal<float> (leftBackRpm,   -speed_max, speed_max);
        rightBackRpm  = aruwlib::algorithms::limitVal<float> (rightBackRpm,  -speed_max, speed_max);
    }

    void ChassisSubsystem::updateMotorRpmPid(
        modm::Pid<float>* pid,
        aruwlib::motor::DjiMotor* const motor,
        float desiredRpm
    ) {
        pid->update(desiredRpm - motor->getShaftRPM());
        motor->setDesiredOutput(pid->getValue());
    }

// todo fix style
    float ChassisSubsystem::chassisSpeedZPID(int16_t errorReal, float kp)
    {
        static int16_t ErrorPrev = 0;
        static int32_t ErrorSum = 0;
        static int32_t ErrorPR = 0;
        static int32_t ErrorPR_KF = 0;
        
        float speed_z = 0;

        ErrorPR_KF = KalmanFilter(&chassisErrorKalman, errorReal);
        
        //P
        speed_z_pterm = errorReal * kp;
        speed_z_pterm = aruwlib::algorithms::limitVal<float>(speed_z_pterm, -REVOLVE_MAX_NORMAL, REVOLVE_MAX_NORMAL);
        
        //I
        ErrorSum -= ErrorPR_KF;
        speed_z_iterm = ErrorSum*3*0.002f; // todo fix this, i in general is messed up
        if( abs(errorReal) <= 10)
        {
            ErrorSum = 0;
        }

        speed_z_iterm = aruwlib::algorithms::limitVal<float>(speed_z_iterm,-5000,5000);
        
        //D
        ErrorPR = ErrorPR_KF - ErrorPrev;
        
        if(abs(ErrorPR_KF) > REVOLVE_ANGLE)
        {
            speed_z_dterm = -(ErrorPR) * REVOLVE_KD;
        }
        else
        {
            speed_z_dterm = 0;
        }

        speed_z = speed_z_pterm + speed_z_dterm; // + speed_i_pterm
        speed_z = aruwlib::algorithms::limitVal<float>(speed_z, -Chassis_Revolve_Move_Max, Chassis_Revolve_Move_Max);

        ErrorPrev = ErrorPR_KF;
        
        return speed_z;
    }

    void ChassisSubsystem::chassisPowerLimit(void)
    {
        bool judgementSystemInvalid = false; // todo
        float currChassisPowerBuffer = 0.0f; // todo
            // run low pass filter on power buffer input
            // run low pass filter on motor current input
        
        // total current output desired, to be compared with current limit
        float allMotorCurrentOutput =
            leftTopMotor.getVoltageDesired()
            + leftBotMotor.getVoltageDesired()
            + rightTopMotor.getVoltageDesired()
            + rightBotMotor.getVoltageDesired();

        float allMotorCurrentLimit;

        if(!judgementSystemInvalid && currChassisPowerBuffer < WARNING_REMAIN_POWER)
        {
            // the total current for all four wheels is limited by the fraction limit
            // the fraction is (power buffer / WARNING_REMAIN_POWER)^2
            // so it will be less than one if WARNING_REMAIN_POWER > power buffer
            // this is limited between 0 and 1 so it can only make the total current
            // smaller
            float chassisPowerFractionLimit = aruwlib::algorithms::limitVal<float>(
                (currChassisPowerBuffer * currChassisPowerBuffer)
                / (WARNING_REMAIN_POWER * WARNING_REMAIN_POWER),
                0.0f, 1.0f
            );
            allMotorCurrentLimit = chassisPowerFractionLimit * CHAS_CURRENT_LIMIT;
        }
        else
        {
            allMotorCurrentLimit = CHAS_CURRENT_LIMIT;
        }

        // limit the chassis output based on fraction between what our current output is
        // and what we realistically can output limited.
        float chassisOutputFraction = allMotorCurrentLimit / allMotorCurrentOutput;
        leftTopMotor.setDesiredOutput(leftTopMotor.getVoltageDesired() * chassisOutputFraction);
        leftBotMotor.setDesiredOutput(leftBotMotor.getVoltageDesired() * chassisOutputFraction);
        rightTopMotor.setDesiredOutput(rightTopMotor.getVoltageDesired() * chassisOutputFraction);
        rightBotMotor.setDesiredOutput(rightBotMotor.getVoltageDesired() * chassisOutputFraction);
    }
}  // namespace control

}  // namespace aruwsrc
