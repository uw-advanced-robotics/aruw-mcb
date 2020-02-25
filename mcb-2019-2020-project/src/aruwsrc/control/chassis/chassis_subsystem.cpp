#include "chassis_subsystem.hpp"
#include "src/aruwlib/algorithms/math_user_utils.hpp"
#include "src/aruwlib/communication/remote.hpp"
#include "src/aruwlib/communication/serial/ref_serial.hpp"

using namespace aruwlib;
using namespace aruwlib::algorithms;

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
        chassisPowerLimit();
    }

    void ChassisSubsystem::mecanumDriveCalculate(float x, float y, float r, float maxWheelSpeed)
    {
        // this is the distance between the center of the chassis to the wheel
        float chassisRotationRatio = sqrtf(powf(WIDTH_BETWEEN_WHEELS_X / 2.0f, 2.0f)
            + powf(WIDTH_BETWEEN_WHEELS_Y / 2.0f, 2.0f));

        // to take into account the location of the turret so we rotate around the turret rather
        // than the center of the chassis, we calculate the offset and than multiply however
        // much we want to rotate by
        float leftFrontRotationRatio
            = degreesToRadians(chassisRotationRatio - GIMBAL_X_OFFSET - GIMBAL_Y_OFFSET);
        float rightFroneRotationRatio
            = degreesToRadians(chassisRotationRatio - GIMBAL_X_OFFSET + GIMBAL_Y_OFFSET);
        float leftBackRotationRatio
            = degreesToRadians(chassisRotationRatio + GIMBAL_X_OFFSET - GIMBAL_Y_OFFSET);
        float rightBackRotationRatio
            = degreesToRadians(chassisRotationRatio + GIMBAL_X_OFFSET + GIMBAL_Y_OFFSET);

        float chassisRotateTranslated = radiansToDegrees(r) / chassisRotationRatio;
        leftFrontRpm  =  y + x + chassisRotateTranslated * leftFrontRotationRatio;
        rightFrontRpm =  y - x + chassisRotateTranslated * rightFroneRotationRatio;
        leftBackRpm   = -y + x + chassisRotateTranslated * leftBackRotationRatio;
        rightBackRpm  = -y - x + chassisRotateTranslated * rightBackRotationRatio;

        leftFrontRpm
            = aruwlib::algorithms::limitVal<float>(leftFrontRpm,  -maxWheelSpeed, maxWheelSpeed);
        rightFrontRpm
            = aruwlib::algorithms::limitVal<float>(rightFrontRpm, -maxWheelSpeed, maxWheelSpeed);
        leftBackRpm
            = aruwlib::algorithms::limitVal<float>(leftBackRpm,   -maxWheelSpeed, maxWheelSpeed);
        rightBackRpm
            = aruwlib::algorithms::limitVal<float>(rightBackRpm,  -maxWheelSpeed, maxWheelSpeed);
    }

    void ChassisSubsystem::updateMotorRpmPid(
        modm::Pid<float>* pid,
        aruwlib::motor::DjiMotor* const motor,
        float desiredRpm
    ) {
        pid->update(desiredRpm - motor->getShaftRPM());
        motor->setDesiredOutput(pid->getValue());
    }

    float ChassisSubsystem::chassisSpeedRotationPID(float currentAngleError, float kp)
    {
        float currentFilteredAngleErrorPrevious = chassisRotationErrorKalman.getLastFiltered();
        float currentFilteredAngleError =
            chassisRotationErrorKalman.filterData(currentAngleError);

        // P
        float currRotationPidP = currentAngleError * kp;
        currRotationPidP = aruwlib::algorithms::limitVal<float>(currRotationPidP,
            -CHASSIS_REVOLVE_PID_MAX_P, CHASSIS_REVOLVE_PID_MAX_P);

        // D
        float currentRotationPidD = 0.0f;
        if(abs(currentFilteredAngleError) > MIN_ERROR_ROTATION_D)
        {
            float currentErrorRotation =
                currentFilteredAngleError - currentFilteredAngleErrorPrevious;

            currentRotationPidD = -(currentErrorRotation) * CHASSIS_REVOLVE_PID_KD;

            currentRotationPidD = aruwlib::algorithms::limitVal<float>(
                currentRotationPidD, -CHASSIS_REVOLVE_PID_MAX_D, CHASSIS_REVOLVE_PID_MAX_D
            );
        }

        float wheelRotationSpeed = aruwlib::algorithms::limitVal<float>(
            currRotationPidP + currentRotationPidD,
            -MAX_WHEEL_SPEED_SINGLE_MOTOR, MAX_WHEEL_SPEED_SINGLE_MOTOR);

        return wheelRotationSpeed;
    }

    float ChassisSubsystem::calculateRotationTranslationalGain(
        float chassisRotationDesiredWheelspeed
    ) {
        // what we will multiply x and y speed by to take into account rotation
        float rTranslationalGain = 1.0f;

        // the x and y movement will be slowed by a fraction of auto rotation amount for maximizing
        // power consumption when the wheel rotation speed for chassis rotationis greater than the
        // MIN_ROTATION_THRESHOLD
        if (fabsf(chassisRotationDesiredWheelspeed) > MIN_ROTATION_THRESHOLD)
        {
            // power(max revolve speed - specified revolve speed, 2)
            // / power(max revolve speed, 2)
            rTranslationalGain =
                powf(
                    ChassisSubsystem::MAX_WHEEL_SPEED_SINGLE_MOTOR + MIN_ROTATION_THRESHOLD
                    - fabsf(chassisRotationDesiredWheelspeed)
                    / ChassisSubsystem::MAX_WHEEL_SPEED_SINGLE_MOTOR,
                    2.0f
                );
            rTranslationalGain
                = aruwlib::algorithms::limitVal<float>(rTranslationalGain, 0.0f, 1.0f);
        }
        return rTranslationalGain;
    }

    float ChassisSubsystem::getChassisX()
    {
        xLowPass = aruwlib::algorithms::lowPassFilter(xLowPass, limitVal<float>(
            Remote::getChannel(Remote::Channel::LEFT_VERTICAL)
            + static_cast<float>(Remote::keyPressed(Remote::Key::W))
            - static_cast<float>(Remote::keyPressed(Remote::Key::S)), -1.0f, 1.0f
        ), REMOTE_LOW_PASS_ALPHA);
        return xLowPass;
    }

    float ChassisSubsystem::getChassisY()
    {
        yLowPass = aruwlib::algorithms::lowPassFilter(yLowPass, limitVal<float>(
            Remote::getChannel(Remote::Channel::LEFT_HORIZONTAL)
            + static_cast<float>(Remote::keyPressed(Remote::Key::A))
            - static_cast<float>(Remote::keyPressed(Remote::Key::D)), -1.0f, 1.0f
        ), REMOTE_LOW_PASS_ALPHA);
        return yLowPass;
    }

    float ChassisSubsystem::getChassisR()
    {
        rLowPass = aruwlib::algorithms::lowPassFilter(rLowPass, limitVal<float>(
            Remote::getChannel(Remote::Channel::RIGHT_HORIZONTAL)
            + static_cast<float>(Remote::keyPressed(Remote::Key::Q))
            - static_cast<float>(Remote::keyPressed(Remote::Key::E)), -1.0f, 1.0f
        ), REMOTE_LOW_PASS_ALPHA);
        return rLowPass;
    }

    void ChassisSubsystem::chassisPowerLimit()
    {
        /// \todo fix this
        bool refereeSystemConnected
            = aruwlib::serial::RefSerial::RefSerial::getRefSerial().isRefSerialOnline();

        float allMotorCurrentLimit;
        float chassisPowerBuffer
            = aruwlib::serial::RefSerial::getRefSerial().getRobotData().chassis.powerBuffer;

        if(refereeSystemConnected && chassisPowerBuffer < MIN_POWER_BUFFER_BEFORE_LIMITING)
        {
            // the total current for all four wheels is limited by the fraction limit
            // the fraction is (power buffer / MIN_POWER_BUFFER_BEFORE_LIMITING)^2
            // so it will be less than one if MIN_POWER_BUFFER_BEFORE_LIMITING > power buffer
            // this is limited between 0 and 1 so it can only make the total current
            // smaller
            float chassisPowerFractionLimit = aruwlib::algorithms::limitVal<float>(
                (chassisPowerBuffer * chassisPowerBuffer)
                / (static_cast<float>(MIN_POWER_BUFFER_BEFORE_LIMITING
                * MIN_POWER_BUFFER_BEFORE_LIMITING)),
                0.0f, 1.0f
            );
            allMotorCurrentLimit = chassisPowerFractionLimit * MAX_TOTAL_CHASSIS_CURRENT;
        }
        else
        {
            allMotorCurrentLimit = MAX_TOTAL_CHASSIS_CURRENT;
        }

        // total current output desired, to be compared with current limit
        float allMotorCurrentOutput =
            abs(leftFrontMotor.getOutputDesired())
            + abs(leftBackMotor.getOutputDesired())
            + abs(rightFrontMotor.getOutputDesired())
            + abs(rightBackMotor.getOutputDesired());
        // limit the chassis output based on fraction between what our current output is
        // and what we realistically can output limited.
        // only limit if the current output is greater than the current limit (what was
        // calculated above)
        if (allMotorCurrentOutput > allMotorCurrentLimit && allMotorCurrentOutput != 0)
        {
            float chassisOutputFraction = allMotorCurrentLimit / allMotorCurrentOutput;
            leftFrontMotor.setDesiredOutput(leftFrontMotor.getOutputDesired()
                    * chassisOutputFraction);
            leftBackMotor.setDesiredOutput(leftBackMotor.getOutputDesired()
                    * chassisOutputFraction);
            rightFrontMotor.setDesiredOutput(rightFrontMotor.getOutputDesired()
                    * chassisOutputFraction);
            rightBackMotor.setDesiredOutput(rightBackMotor.getOutputDesired()
                    * chassisOutputFraction);
        }
    }
}  // namespace chassis

}  // namespace aruwsrc
