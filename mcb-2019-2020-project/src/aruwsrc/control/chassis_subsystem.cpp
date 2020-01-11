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
        updateMotorRpmPid(&leftFrontVelocityPid, &leftFrontMotor, leftFrontRpm);
        updateMotorRpmPid(&leftBackVelocityPid, &leftBackMotor, leftBackRpm);
        updateMotorRpmPid(&rightFrontVelocityPid, &rightFrontMotor, rightFrontRpm);
        updateMotorRpmPid(&rightBackVelocityPid, &rightBackMotor, rightBackRpm);
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

        errorPRotateKalman = KalmanFilter(&chassisErrorKalman, errorReal);

        // P
        rotationPidP = errorReal * kp;
        rotationPidP = aruwlib::algorithms::limitVal<float>(rotationPidP,
            -CHASSIS_REVOLVE_PID_MAX_P, CHASSIS_REVOLVE_PID_MAX_P);

        // D
        errorPRotate = errorPRotateKalman - errorPrevKalman;

        if(abs(errorPRotateKalman) > MAX_REVOLVE_ANGLE)
        {
            rotationPidD = -(errorPRotate) * CHASSIS_REVOLVE_PID_KD;
        }
        else
        {
            rotationPidD = 0;
        }

        speed_z = rotationPidP + rotationPidD;
        speed_z = aruwlib::algorithms::limitVal<float>(speed_z, -OMNI_SPEED_MAX, OMNI_SPEED_MAX);

        errorPrevKalman = errorPRotateKalman;

        return speed_z;
    }
}  // namespace control

}  // namespace aruwsrc
