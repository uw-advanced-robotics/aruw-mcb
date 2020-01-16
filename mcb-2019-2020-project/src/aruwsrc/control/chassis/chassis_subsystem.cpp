#include "chassis_subsystem.hpp"
#include "src/aruwlib/algorithms/math_user_utils.hpp"
#include "src/aruwlib/communication/remote.hpp"

using namespace aruwlib;

namespace aruwsrc
{

namespace chassis
{
    void ChassisSubsystem::setDesiredOutput(float x, float y, float r)
    {
        mecanumDriveCalculate(x, y, r, MAX_WHEEL_SPEED_SINGLE_MOTOR);
    }

    void ChassisSubsystem::refresh()
    {
        updateMotorRpmPid(&leftFrontVelocityPid, &leftFrontMotor, leftFrontRpm);
        updateMotorRpmPid(&leftBackVelocityPid, &leftBackMotor, leftBackRpm);
        updateMotorRpmPid(&rightFrontVelocityPid, &rightFrontMotor, rightFrontRpm);
        updateMotorRpmPid(&rightBackVelocityPid, &rightBackMotor, rightBackRpm);
    }

    void ChassisSubsystem::mecanumDriveCalculate(float x, float y, float r, float maxWheelSpeed)
    {
        float chassisRotationRatio = (WHEELBASE + WHEELTRACK) / 2.0f;

        float rotateRatioFL = DEGREES_TO_RADIANS(
            chassisRotationRatio - GIMBAL_X_OFFSET - GIMBAL_Y_OFFSET);
        float rotateRatioRF = DEGREES_TO_RADIANS(
            chassisRotationRatio - GIMBAL_X_OFFSET + GIMBAL_Y_OFFSET);
        float rotateRatioBL = DEGREES_TO_RADIANS(
            chassisRotationRatio + GIMBAL_X_OFFSET - GIMBAL_Y_OFFSET);
        float rotateRatioBR = DEGREES_TO_RADIANS(
            chassisRotationRatio + GIMBAL_X_OFFSET + GIMBAL_Y_OFFSET);

        float chassisRotateTranslated = RADIANS_TO_DEGREES(r) / chassisRotationRatio;
        leftFrontRpm  =  ( y + x + chassisRotateTranslated * rotateRatioFL);
        rightFrontRpm = -(-y + x - chassisRotateTranslated * rotateRatioRF);
        leftBackRpm   =  (-y + x + chassisRotateTranslated * rotateRatioBL);
        rightBackRpm  = -( y + x - chassisRotateTranslated * rotateRatioBR);

        leftFrontRpm  = aruwlib::algorithms::limitVal<float> (leftFrontRpm,  -maxWheelSpeed, maxWheelSpeed);
        rightFrontRpm = aruwlib::algorithms::limitVal<float> (rightFrontRpm, -maxWheelSpeed, maxWheelSpeed);
        leftBackRpm   = aruwlib::algorithms::limitVal<float> (leftBackRpm,   -maxWheelSpeed, maxWheelSpeed);
        rightBackRpm  = aruwlib::algorithms::limitVal<float> (rightBackRpm,  -maxWheelSpeed, maxWheelSpeed);
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
        float wheelRotationSpeed = 0;

        errorPRotateKalman = KalmanFilter(&chassisErrorKalman, currentAngleError);

        // P
        rotationPidP = currentAngleError * kp;
        rotationPidP = aruwlib::algorithms::limitVal<float>(rotationPidP,
            -CHASSIS_REVOLVE_PID_MAX_P, CHASSIS_REVOLVE_PID_MAX_P);

        // D
        errorPRotate = errorPRotateKalman - errorPrevKalman;

        if(abs(errorPRotateKalman) > MIN_ERROR_ROTATION_D)
        {
            rotationPidD = -(errorPRotate) * CHASSIS_REVOLVE_PID_KD;
        }
        else
        {
            rotationPidD = 0.0f;
        }

        wheelRotationSpeed = rotationPidP + rotationPidD;
        wheelRotationSpeed = aruwlib::algorithms::limitVal<float>(wheelRotationSpeed,
            -MAX_WHEEL_SPEED_SINGLE_MOTOR, MAX_WHEEL_SPEED_SINGLE_MOTOR);

        errorPrevKalman = errorPRotateKalman;

        return wheelRotationSpeed;
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

    float ChassisSubsystem::getChassisR()
    {
        return aruwlib::algorithms::limitVal<float>(
            Remote::getChannel(Remote::Channel::RIGHT_HORIZONTAL)
            + static_cast<float>(Remote::keyPressed(Remote::Key::Q)
            - Remote::keyPressed(Remote::Key::E)), -1.0f, 1.0f
        );
    }
}  // namespace chassis

}  // namespace aruwsrc
