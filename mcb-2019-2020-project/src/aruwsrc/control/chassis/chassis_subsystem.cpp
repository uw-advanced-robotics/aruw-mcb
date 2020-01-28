#include "chassis_subsystem.hpp"
#include "src/aruwlib/algorithms/math_user_utils.hpp"
#include "src/aruwlib/communication/remote.hpp"
#include "src/main.hpp"

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
    }

    float ChassisSubsystem::currentControl()
    {
        // max allowed power is 80 W, Battery is 24V. Max current allowed is 80/24
        // -(log base (80/24)^2 (80/24) x + 2). When current is at max, scale acceleration by 1. 
        // buffer of a^2/b^2 = (80/24)^2 logarithmic base
        // + 2 so nonnegative values when current is capped
        // scales down high current values
        // if current < 80/24, scale by larger constant based on gap in current
        // add cap: 1.5?
        // add lower limit: 0.5?

        // visualization of graph on desmos: https://www.desmos.com/calculator/hjec1vpo1z

        // Problem: current is improperly constrained. Soldier life bar continues to deplete. 
        // Could be related to refSerial not returning anything (wiring issue), problem with 
        // constants, or something else

        
        float chassisCurrent = refSerial.getRobotData().chassis.current / 1000.0f;

        if (chassisCurrent < 0.00001f) {
            chassisCurrent = 5.0f;
        }

        float currentMultiplier = -(log10f(80.0f / 24.0f * chassisCurrent) / log10f(powf((80.0f /24.0f ), 2.0f))) + 1.5f;

        if(currentMultiplier > 1.0f)
        {
            currentMultiplier = 1.0f;
        }

        // add lower limit:
        if(currentMultiplier < 0.0f)
        {
            currentMultiplier = 0.0f;
        }

        return currentMultiplier;
    }

    void ChassisSubsystem::mecanumDriveCalculate(float x, float y, float r, float maxWheelSpeed)
    {
        // this is the distance between the center of the chassis to the wheel
        float chassisRotationRatio = (WIDTH_BETWEEN_WHEELS_X + WIDTH_BETWEEN_WHEELS_Y) / 2.0f;

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
        //update here; multiply desiredOutput by currentControl()
        motor->setDesiredOutput(currentControl() * pid->getValue());
    }

    float ChassisSubsystem::chassisSpeedRotationPID(float currentAngleError, float kp)
    {
        float currentFilteredAngleErrorPrevious = chassisRotationErrorKalman.getLastPrediction();
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
        if (fabs(chassisRotationDesiredWheelspeed) > MIN_ROTATION_THRESHOLD)
        {
            // power(max revolve speed - specified revolve speed, 2)
            // / power(max revolve speed, 2)
            rTranslationalGain =
                pow(
                    static_cast<double>(
                    ChassisSubsystem::MAX_WHEEL_SPEED_SINGLE_MOTOR + MIN_ROTATION_THRESHOLD
                    - fabs(chassisRotationDesiredWheelspeed)
                    / ChassisSubsystem::MAX_WHEEL_SPEED_SINGLE_MOTOR),
                    2.0
                );
            rTranslationalGain
                = aruwlib::algorithms::limitVal<float>(rTranslationalGain, 0.0f, 1.0f);
        }
        return rTranslationalGain;
    }

    float ChassisSubsystem::getChassisX()
    {
        return aruwlib::algorithms::limitVal<float>(
            Remote::getChannel(Remote::Channel::LEFT_VERTICAL)
            + static_cast<float>(Remote::keyPressed(Remote::Key::W))
            - static_cast<float>(Remote::keyPressed(Remote::Key::S)), -1.0f, 1.0f
        );
    }

    float ChassisSubsystem::getChassisY()
    {
        return aruwlib::algorithms::limitVal<float>(
            Remote::getChannel(Remote::Channel::LEFT_HORIZONTAL)
            + static_cast<float>(Remote::keyPressed(Remote::Key::A))
            - static_cast<float>(Remote::keyPressed(Remote::Key::D)), -1.0f, 1.0f
        );
    }

    float ChassisSubsystem::getChassisR()
    {
        return aruwlib::algorithms::limitVal<float>(
            Remote::getChannel(Remote::Channel::RIGHT_HORIZONTAL)
            + static_cast<float>(Remote::keyPressed(Remote::Key::Q))
            - static_cast<float>(Remote::keyPressed(Remote::Key::E)), -1.0f, 1.0f
        );
    }
}  // namespace chassis

}  // namespace aruwsrc
