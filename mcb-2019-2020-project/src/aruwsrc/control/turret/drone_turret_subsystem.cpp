#include <aruwlib/algorithms/math_user_utils.hpp>
#include <aruwlib/control/controller_mapper.hpp>
#include <aruwlib/errors/create_errors.hpp>
#include "drone_turret_subsystem.hpp"
#include <aruwlib/communication/sensors/mpu6500/mpu6500.hpp>

using namespace aruwlib;
using namespace aruwlib::motor;

namespace aruwsrc
{

namespace turret
{
DroneTurretSubsystem::DroneTurretSubsystem() :
    pitchMotor(PITCH_MOTOR_ID, CAN_BUS_MOTORS, true, "pitch motor"),
    yawMotor(YAW_MOTOR_ID, CAN_BUS_MOTORS, true, "yaw motor"),
    currPitchAngle(0.0f, 0.0f, 360.0f),
    currYawAngle(0.0f, 0.0f, 360.0f),
    yawTarget(TURRET_YAW_START_ANGLE, 0.0f, 360.0f),
    pitchTarget(TURRET_PITCH_START_ANGLE, 0.0f, 360.0f)
{}

float DroneTurretSubsystem::getYawAngleFromCenter() const
{
    aruwlib::algorithms::ContiguousFloat yawAngleFromCenter(
        currYawAngle.getValue() - TURRET_YAW_START_ANGLE, -180.0f, 180.0f);
    return yawAngleFromCenter.getValue();
}

float DroneTurretSubsystem::getPitchAngleFromCenter() const
{
    aruwlib::algorithms::ContiguousFloat pitchAngleFromCenter(
        currPitchAngle.getValue() - TURRET_PITCH_START_ANGLE, -180.0f, 180.0f);
    return pitchAngleFromCenter.getValue();
}

float DroneTurretSubsystem::getYawVelocity() const
{
    if (!yawMotor.isMotorOnline())
    {
        RAISE_ERROR("trying to get velocity and yaw motor offline",
                aruwlib::errors::TURRET, aruwlib::errors::MOTOR_OFFLINE);
        // throw error
        return 0;
    }

    return getVelocity(yawMotor);
}

float DroneTurretSubsystem::getPitchVelocity() const
{
    if (!pitchMotor.isMotorOnline())
    {
        RAISE_ERROR("trying to get velocity and pitch motor offline",
                aruwlib::errors::TURRET, aruwlib::errors::MOTOR_OFFLINE);
        return 0;
    }

    return getVelocity(pitchMotor);
}

// units: degrees per second
float DroneTurretSubsystem::getVelocity(const DjiMotor &motor) const
{
    return 360.0f / 60.0f * motor.getShaftRPM();
}

bool DroneTurretSubsystem::isTurretOnline() const
{
    return pitchMotor.isMotorOnline() && yawMotor.isMotorOnline();
}

void DroneTurretSubsystem::refresh()
{
    updateCurrentTurretAngles();
}
float mpu6500Pitch = 0.0f;
float mpu6500Gx = 0.0f;
float pitchMotorSpeed = 0.0f;
void DroneTurretSubsystem::updateCurrentTurretAngles()
{
    updateCurrentYawAngle();
    updateCurrentPitchAngle();
    mpu6500Pitch = -aruwlib::sensors::Mpu6500::getImuAttitude().roll;
    mpu6500Gx = -aruwlib::sensors::Mpu6500::getGx();
    pitchMotorSpeed = getPitchVelocity();
}

void DroneTurretSubsystem::updateCurrentYawAngle()
{
    if (yawMotor.isMotorOnline())
    {
        currYawAngle.setValue(DjiMotor::encoderToDegrees(static_cast<uint16_t>(
                yawMotor.encStore.getEncoderWrapped() - YAW_START_ENCODER_POSITION))
                + TURRET_YAW_START_ANGLE);
    }
    else
    {
        currYawAngle.setValue(TURRET_YAW_START_ANGLE);
    }
}

void DroneTurretSubsystem::updateCurrentPitchAngle()
{
    if (pitchMotor.isMotorOnline())
    {
        currPitchAngle.setValue(DjiMotor::encoderToDegrees(static_cast<uint16_t>(
                pitchMotor.encStore.getEncoderWrapped() - PITCH_START_ENCODER_POSITION))
                + TURRET_PITCH_START_ANGLE);
        currPitchAngle.setValue(currPitchAngle.getValue());
    }
    else
    {
        currPitchAngle.setValue(TURRET_PITCH_START_ANGLE);
    }
}

void DroneTurretSubsystem::setPitchMotorOutput(float out)
{
    if (out > INT32_MAX || out < INT32_MIN)
    {
        RAISE_ERROR("pitch motor output invalid",
                aruwlib::errors::TURRET, aruwlib::errors::INVALID_MOTOR_OUTPUT);
        return;
    }
    if (pitchMotor.isMotorOnline())
    {
        if (getPitchAngleFromCenter() + TURRET_PITCH_START_ANGLE >
                TURRET_PITCH_MAX_ANGLE - TURRET_DEADBAND && out > 0) {
            float a = getPitchAngleFromCenter() + TURRET_PITCH_START_ANGLE - TURRET_PITCH_MAX_ANGLE + TURRET_DEADBAND;
            a = (a > TURRET_DEADBAND ? 0 : (TURRET_DEADBAND - a) * TURRET_DEADBAND_DECAY_COEFFICIENT);
            a = (a < 0.0f ? 0.0f : a);
            pitchMotor.setDesiredOutput(a * out);
        } else if (getPitchAngleFromCenter() + TURRET_PITCH_START_ANGLE <
                TURRET_PITCH_MIN_ANGLE + TURRET_DEADBAND && out < 0)
        {
            float a = TURRET_PITCH_MIN_ANGLE + TURRET_DEADBAND - getPitchAngleFromCenter() - TURRET_PITCH_START_ANGLE;
            a = (a > TURRET_DEADBAND ? 0 : (TURRET_DEADBAND - a) * TURRET_DEADBAND_DECAY_COEFFICIENT);
            a = (a < 0.0f ? 0.0f : a);
            pitchMotor.setDesiredOutput(a * out);
        }
        else
        {
            pitchMotor.setDesiredOutput(out);
        }
    }
}

void DroneTurretSubsystem::setYawMotorOutput(float out)
{
    if (out > INT32_MAX || out < INT32_MIN) {
        RAISE_ERROR("yaw motor output invalid",
                aruwlib::errors::TURRET, aruwlib::errors::INVALID_MOTOR_OUTPUT);
        return;
    }
    if (yawMotor.isMotorOnline())
    {
        if (getYawAngleFromCenter() + TURRET_YAW_START_ANGLE >
                TURRET_YAW_MAX_ANGLE - TURRET_DEADBAND && out > 0) {
            float a = (getYawAngleFromCenter() + TURRET_YAW_START_ANGLE) - (TURRET_YAW_MAX_ANGLE - TURRET_DEADBAND);
            a = (a > TURRET_DEADBAND ? 0 : (TURRET_DEADBAND - a) * TURRET_DEADBAND_DECAY_COEFFICIENT);
            a = (a < 0.0f ? 0.0f : a);
            yawMotor.setDesiredOutput(a * out);
        } else if (getYawAngleFromCenter() + TURRET_YAW_START_ANGLE <
                TURRET_YAW_MIN_ANGLE + TURRET_DEADBAND && out < 0)
        {
            float a = (TURRET_YAW_MIN_ANGLE + TURRET_DEADBAND) - (getYawAngleFromCenter() + TURRET_YAW_START_ANGLE);
            a = (a >= TURRET_DEADBAND ? 0 : (TURRET_DEADBAND - a) * TURRET_DEADBAND_DECAY_COEFFICIENT);
            a = (a < 0.0f ? 0.0f : a);
            yawMotor.setDesiredOutput(a * out);
        }
        else
        {
            yawMotor.setDesiredOutput(out);
        }
    }
}

const aruwlib::algorithms::ContiguousFloat& DroneTurretSubsystem::getYawAngle() const
{
    return currYawAngle;
}

const aruwlib::algorithms::ContiguousFloat& DroneTurretSubsystem::getPitchAngle() const
{
    return currPitchAngle;
}

float DroneTurretSubsystem::getYawTarget() const
{
    return yawTarget.getValue();
}

float DroneTurretSubsystem::getPitchTarget() const
{
    return pitchTarget.getValue();
}

void DroneTurretSubsystem::setYawTarget(float target)
{
    yawTarget.setValue(target);
    yawTarget.setValue(aruwlib::algorithms::ContiguousFloat::limitValue(yawTarget,
            TURRET_YAW_MIN_ANGLE, TURRET_YAW_MAX_ANGLE));
}

void DroneTurretSubsystem::setPitchTarget(float target)
{
    pitchTarget.setValue(target);
    pitchTarget.setValue(aruwlib::algorithms::ContiguousFloat::limitValue(pitchTarget,
            TURRET_PITCH_MIN_ANGLE, TURRET_PITCH_MAX_ANGLE));
}



float DroneTurretSubsystem::yawFeedForwardCalculation()
{
    /*
    // calculate feed forward
    float chassisRotationProportional = FEED_FORWARD_KP
            * desiredChassisRotation
            * (fabsf(FEED_FORWARD_SIN_GAIN * sinf(getYawAngleFromCenter()
            * aruwlib::algorithms::PI / 180.0f)) + 1.0f);

    if (Remote::getUpdateCounter() != prevUpdateCounterChassisRotateDerivative) {
        chassisRotateDerivativeInterpolation.update(
                desiredChassisRotation - feedforwardPrevChassisRotationDesired);
    }
    prevUpdateCounterChassisRotateDerivative = Remote::getUpdateCounter();
    float derivativeInterpolated = chassisRotateDerivativeInterpolation
            .getInterpolatedValue(modm::Clock::now().getTime());

    feedforwardChassisRotateDerivative = aruwlib::algorithms::lowPassFilter(
            feedforwardChassisRotateDerivative,
            derivativeInterpolated,
            FEED_FORWARD_DERIVATIVE_LOW_PASS);

    float chassisRotationFeedForward = aruwlib::algorithms::limitVal<float>(
            chassisRotationProportional + FEED_FORWARD_KD * feedforwardChassisRotateDerivative,
            -FEED_FORWARD_MAX_OUTPUT, FEED_FORWARD_MAX_OUTPUT);

    feedforwardPrevChassisRotationDesired = desiredChassisRotation;

    if ((chassisRotationFeedForward > 0.0f
        && getYawAngle().getValue() > DroneTurretSubsystem::TURRET_YAW_MAX_ANGLE)
        || (chassisRotationFeedForward < 0.0f
        && getYawAngle().getValue() < DroneTurretSubsystem::TURRET_YAW_MIN_ANGLE))
    {
        chassisRotationFeedForward = 0.0f;
    }
    return chassisRotationFeedForward;
    */
    return 0.0f;
}
}  // namespace turret

}  // namespace aruwsrc
