#include "chassis_subsystem.hpp"
#include "src/aruwlib/algorithms/math_user_utils.hpp"

namespace aruwsrc
{

namespace control
{
    void ChassisSubsystem::setDesiredOutput(float x, float y, float z)
    {
        chassisOmniMoveCalculate(x, y, z, OMNI_SPEED_MAX);
    }

    void ChassisSubsystem::refresh()
    {
        updateMotorRpmPid(&leftTopVelocityPid, &leftTopMotor, leftFrontRpm);
        updateMotorRpmPid(&leftBotVelocityPid, &leftBotMotor, leftBackRpm);
        updateMotorRpmPid(&rightTopVelocityPid, &rightTopMotor, rightFrontRpm);
        updateMotorRpmPid(&rightBotVelocityPid, &rightBotMotor, rightBackRpm);
    }

    void ChassisSubsystem::chassisOmniMoveCalculate(float x, float y, float z, float speedMax)
    {
        float rotateRatioFL, rotateRatioRF, rotateRatioBL, rotateRatioBR;
        float chassisRotationRatio = (WHEELBASE + WHEELTRACK) / 2.0f;

        rotateRatioFL = DEGREES_TO_RADIANS(
            chassisRotationRatio - GIMBAL_X_OFFSET - GIMBAL_Y_OFFSET);
        rotateRatioRF = DEGREES_TO_RADIANS(
            chassisRotationRatio - GIMBAL_X_OFFSET + GIMBAL_Y_OFFSET);
        rotateRatioBL = DEGREES_TO_RADIANS(
            chassisRotationRatio + GIMBAL_X_OFFSET - GIMBAL_Y_OFFSET);
        rotateRatioBR = DEGREES_TO_RADIANS(
            chassisRotationRatio + GIMBAL_X_OFFSET + GIMBAL_Y_OFFSET);

        float zTrans = RADIANS_TO_DEGREES(z) / chassisRotationRatio;
        leftFrontRpm  =  ( y + x + zTrans * rotateRatioFL);
        rightFrontRpm = -(-y + x - zTrans * rotateRatioRF);
        leftBackRpm   =  (-y + x + zTrans * rotateRatioBL);
        rightBackRpm  = -( y + x - zTrans * rotateRatioBR);

        leftFrontRpm  = aruwlib::algorithms::limitVal<float> (leftFrontRpm,  -speedMax, speedMax);
        rightFrontRpm = aruwlib::algorithms::limitVal<float> (rightFrontRpm, -speedMax, speedMax);
        leftBackRpm   = aruwlib::algorithms::limitVal<float> (leftBackRpm,   -speedMax, speedMax);
        rightBackRpm  = aruwlib::algorithms::limitVal<float> (rightBackRpm,  -speedMax, speedMax);
    }

    void ChassisSubsystem::updateMotorRpmPid(
        modm::Pid<float>* pid,
        aruwlib::motor::DjiMotor* const motor,
        float desiredRpm
    ) {
        pid->update(desiredRpm - motor->getShaftRPM());
        motor->setDesiredOutput(pid->getValue());
    }

    float ChassisSubsystem::chassisSpeedZPID(float errorReal, float kp)
    {
        float speed_z = 0;

        ErrorPR_KF = KalmanFilter(&chassisErrorKalman, errorReal);

        // P
        rotationPidP = errorReal * kp;
        rotationPidP = aruwlib::algorithms::limitVal<float>(rotationPidP,
            -CHASSIS_REVOLVE_PID_MAX_P, CHASSIS_REVOLVE_PID_MAX_P);

        // D
        ErrorPR = ErrorPR_KF - ErrorPrev;

        if(abs(ErrorPR_KF) > MAX_REVOLVE_ANGLE)
        {
            rotationPidD = -(ErrorPR) * CHASSIS_REVOLVE_PID_KD;
        }
        else
        {
            rotationPidD = 0;
        }

        speed_z = rotationPidP + rotationPidD;
        speed_z = aruwlib::algorithms::limitVal<float>(speed_z, -OMNI_SPEED_MAX, OMNI_SPEED_MAX);

        ErrorPrev = ErrorPR_KF;

        return speed_z;
    }

    void ChassisSubsystem::chassisPowerLimit(void)
    {
        bool judgementSystemInvalid = false;  // todo
        float currChassisPowerBuffer = 0.0f;  // todo
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
