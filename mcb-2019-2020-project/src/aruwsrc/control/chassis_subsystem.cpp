#include "chassis_subsystem.hpp"
#include "src/aruwlib/algorithms/math_user_utils.hpp"

#define DEG_TO_RAD(val) (val * 3.14f / 180.0f)
#define PI (3.14f)

namespace aruwsrc
{

namespace control
{
    const aruwlib::motor::MotorId ChassisSubsystem::LEFT_TOP_MOTOR_ID = aruwlib::motor::MOTOR4;
    const aruwlib::motor::MotorId ChassisSubsystem::LEFT_BOT_MOTOR_ID = aruwlib::motor::MOTOR5;
    const aruwlib::motor::MotorId ChassisSubsystem::RIGHT_TOP_MOTOR_ID = aruwlib::motor::MOTOR6;
    const aruwlib::motor::MotorId ChassisSubsystem::RIGHT_BOT_MOTOR_ID = aruwlib::motor::MOTOR7;

    void ChassisSubsystem::setDesiredOutput(float x, float y, float r)
    {
        // TODO calculate motor rpm values based on given x, y, and r
        chassisOmniMoveCalculate(x, y, r);
    }

    void ChassisSubsystem::refresh()
    {
        updateMotorRpmPid(&leftTopVelocityPid, &leftTopMotor, leftTopRpm);
        updateMotorRpmPid(&leftBotVelocityPid, &leftBotMotor, leftBotRpm);
        updateMotorRpmPid(&rightTopVelocityPid, &rightTopMotor, rightTopRpm);
        updateMotorRpmPid(&rightBotVelocityPid, &rightBotMotor, rightBotRpm);
    }

    void ChassisSubsystem::chassisOmniMoveCalculate(float x, float y, float z)
    {
        float rotate_ratio_fl, rotate_ratio_fr, rotate_ratio_bl, rotate_ratio_br;
        float wheel_rpm_ratio;
        float speed_max;

        rotate_ratio_fr = ((WHEELBASE + WHEELTRACK) / 2.0f - GIMBAL_X_OFFSET + GIMBAL_Y_OFFSET) / RADIAN_COEF;
        rotate_ratio_fl = ((WHEELBASE + WHEELTRACK) / 2.0f - GIMBAL_X_OFFSET - GIMBAL_Y_OFFSET) / RADIAN_COEF;
        rotate_ratio_bl = ((WHEELBASE + WHEELTRACK) / 2.0f + GIMBAL_X_OFFSET - GIMBAL_Y_OFFSET) / RADIAN_COEF;
        rotate_ratio_br = ((WHEELBASE + WHEELTRACK) / 2.0f + GIMBAL_X_OFFSET + GIMBAL_Y_OFFSET) / RADIAN_COEF;

        wheel_rpm_ratio = 60.0f / (PERIMETER * CHASSIS_GEARBOX_RATIO); // what is this
        
        float zTrans = z / (((WHEELBASE + WHEELTRACK) / 2.0f) / RADIAN_COEF);

        leftTopRpm  =  ( y + x + zTrans) / wheel_rpm_ratio * rotate_ratio_fl;
        rightTopRpm = -(-y + x - zTrans) / wheel_rpm_ratio * rotate_ratio_fr;
        leftBotRpm  =  (-y + x + zTrans) / wheel_rpm_ratio * rotate_ratio_bl;
        rightBotRpm = -( y + x - zTrans) / wheel_rpm_ratio * rotate_ratio_br;

        leftTopRpm  = aruwlib::algorithms::limitVal<float> (leftTopRpm,  -speed_max, speed_max);
        rightTopRpm = aruwlib::algorithms::limitVal<float> (rightTopRpm, -speed_max, speed_max);
        leftBotRpm  = aruwlib::algorithms::limitVal<float> (leftBotRpm,  -speed_max, speed_max);
        rightBotRpm = aruwlib::algorithms::limitVal<float> (rightBotRpm, -speed_max, speed_max);
    }

    void ChassisSubsystem::updateMotorRpmPid(
        modm::Pid<float>* pid,
        aruwlib::motor::DjiMotor* const motor,
        float desiredRpm
    ) {
        pid->update(desiredRpm - motor->getShaftRPM());
        motor->setDesiredOutput(pid->getValue());
    }

    float ChassisSubsystem::Chassis_SpeedZ_PID(int16_t ErrorReal, float kp)
    {
        static int16_t ErrorPrev = 0;
        static int32_t ErrorSum = 0;
        static int32_t ErrorPR = 0;
        static int32_t ErrorPR_KF = 0;
        
        float speed_z = 0;

        ErrorPR_KF = KalmanFilter(&chassisErrorKalman, ErrorReal);
        
        //P
        speed_z_pterm = ErrorReal * kp;
        speed_z_pterm = aruwlib::algorithms::limitVal<float>(speed_z_pterm, -REVOLVE_MAX_NORMAL, REVOLVE_MAX_NORMAL);
        
        //I
        ErrorSum -= ErrorPR_KF;
        speed_z_iterm = ErrorSum*3*0.002f; // todo fix this, i in general is messed up
        if( abs(ErrorReal) <= 10)
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

        speed_z = speed_z_pterm + speed_z_dterm;
        speed_z = aruwlib::algorithms::limitVal<float>(speed_z, -Chassis_Revolve_Move_Max, +Chassis_Revolve_Move_Max);

        ErrorPrev = ErrorPR_KF;
        
        return speed_z;
    }

    void ChassisSubsystem::Chassis_Power_Limit(void)
    {	
        // float    chassis_totaloutput = 0;
        // float    Joule_Residue = 0;
        // bool  judgDataCorrect = 0;	
        // static int32_t judgDataError_Time = 0;
        // float fTotalCurrentLimit;

        bool judgDataCorrect = false; // todo
        float Joule_Residue = 0.0f; // todo	
        
        float chassis_totaloutput =
            leftTopMotor.getVoltageDesired()
            + leftBotMotor.getVoltageDesired()
            + rightTopMotor.getVoltageDesired()
            + rightBotMotor.getVoltageDesired();

        float fTotalCurrentLimit;

        if(!judgDataCorrect)
        {
            fTotalCurrentLimit = 9000; // todo fix
        }
        else
        {
            if(Joule_Residue < WARNING_REMAIN_POWER)
            {
                float kLimit = (Joule_Residue / WARNING_REMAIN_POWER)
                    * (Joule_Residue / WARNING_REMAIN_POWER);
                
                fTotalCurrentLimit = kLimit * fChasCurrentLimit;
            }
            else
            {
                fTotalCurrentLimit = fChasCurrentLimit;
            }
        }

        if (chassis_totaloutput > fTotalCurrentLimit)
        {
            leftTopMotor.setDesiredOutput(leftTopMotor.getVoltageDesired()
                / chassis_totaloutput * fTotalCurrentLimit);
            leftBotMotor.setDesiredOutput(leftBotMotor.getVoltageDesired()
                / chassis_totaloutput * fTotalCurrentLimit);
            rightTopMotor.setDesiredOutput(rightTopMotor.getVoltageDesired()
                / chassis_totaloutput * fTotalCurrentLimit);
            rightBotMotor.setDesiredOutput(rightBotMotor.getVoltageDesired()
                / chassis_totaloutput * fTotalCurrentLimit);	
        }
    }

}  // namespace control

}  // namespace aruwsrc