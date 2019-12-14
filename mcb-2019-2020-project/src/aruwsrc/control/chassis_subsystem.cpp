#include "chassis_subsystem.hpp"
#include "src/aruwlib/algorithms/math_user_utils.hpp"

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
        leftTopRpm = 0;
        leftBotRpm = 0;
        rightTopRpm = 0;
        rightBotRpm = 0;
    }

    void ChassisSubsystem::refresh()
    {
        updateMotorRpmPid(&leftTopVelocityPid, &leftTopMotor, leftTopRpm);
        updateMotorRpmPid(&leftBotVelocityPid, &leftBotMotor, leftBotRpm);
        updateMotorRpmPid(&rightTopVelocityPid, &rightTopMotor, rightTopRpm);
        updateMotorRpmPid(&rightBotVelocityPid, &rightBotMotor, rightBotRpm);
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
        static int16_t ErrorPrev = 0;//�ϴ�ƫ�����
        static int32_t ErrorSum = 0;//�ϴ�ƫ�����
        static int32_t ErrorPR = 0;
        static int32_t ErrorPR_KF = 0;
        
        float speed_z = 0;

        ErrorPR_KF = KalmanFilter(&chassisErrorKalman, ErrorReal);
        
        //P
        speed_z_pterm = ErrorReal * kp;//����yawƫ�����ļ������
        speed_z_pterm = aruwlib::algorithms::limitVal<float>(speed_z_pterm, -REVOLVE_MAX_NORMAL, REVOLVE_MAX_NORMAL);
        
        //I
        ErrorSum -= ErrorPR_KF;
        speed_z_iterm = ErrorSum*3*0.002f; // todo fix this
        if( abs(ErrorReal) <= 10)
        {
            ErrorSum = 0;
        }
        //�����޷�
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
        //Ťͷ����ٶ��޷�
        speed_z = speed_z_pterm + speed_z_dterm;
        speed_z = aruwlib::algorithms::limitVal<float>(speed_z, -Chassis_Revolve_Move_Max, +Chassis_Revolve_Move_Max);

        ErrorPrev = ErrorPR_KF;//��¼�ϴ����
        
        return speed_z;
    }

}  // namespace control

}  // namespace aruwsrc