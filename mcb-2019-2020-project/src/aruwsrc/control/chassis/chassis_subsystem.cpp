#include "chassis_subsystem.hpp"
#include "src/aruwlib/algorithms/math_user_utils.hpp"
#include "src/aruwlib/communication/remote.hpp"

using namespace aruwlib;

namespace aruwsrc
{

namespace chassis
{
    void ChassisSubsystem::setDesiredOutput(float x, float y, float z)
    {
        chassisOmniMoveCalculate(x, y, z, MAX_CURRENT_OUT_SINGLE_MOTOR);
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

    float ChassisSubsystem::chassisSpeedZPID(float currentAngleError, float kp)
    {
        float speed_z = 0;

        errorPRotateKalman = KalmanFilter(&chassisErrorKalman, currentAngleError);

        // P
        rotationPidP = currentAngleError * kp;
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
            rotationPidD = 0.0f;
        }

        speed_z = rotationPidP + rotationPidD;
        speed_z = aruwlib::algorithms::limitVal<float>(speed_z,
            -MAX_CURRENT_OUT_SINGLE_MOTOR, MAX_CURRENT_OUT_SINGLE_MOTOR);

        errorPrevKalman = errorPRotateKalman;

        return speed_z;
    }

    float ChassisSubsystem::getChassisX()
    {
        return aruwlib::algorithms::limitVal<float>(
            Remote::getChannel(Remote::Channel::LEFT_VERTICAL)
            + static_cast<float>(Remote::keyPressed(Remote::Key::W)
            - Remote::keyPressed(Remote::Key::S)), -1.0f, 1.0f
        );
    }

    float ChassisSubsystem::getChassisY()
    {
        return aruwlib::algorithms::limitVal<float>(
            Remote::getChannel(Remote::Channel::LEFT_HORIZONTAL)
            + static_cast<float>(Remote::keyPressed(Remote::Key::A)
            - Remote::keyPressed(Remote::Key::D)), -1.0f, 1.0f
        );
    }

    float ChassisSubsystem::getChassisZ()
    {
        return aruwlib::algorithms::limitVal<float>(
            Remote::getChannel(Remote::Channel::RIGHT_HORIZONTAL)
            + static_cast<float>(Remote::keyPressed(Remote::Key::Q)
            - Remote::keyPressed(Remote::Key::E)), -1.0f, 1.0f
        );
    }
}  // namespace chassis

}  // namespace aruwsrc
