#include <algorithm>
#include <random>
#include <cfloat>
#include "turret_subsystem.hpp"
#include "src/aruwlib/algorithms/math_user_utils.hpp"
#include "src/aruwlib/control/controller_mapper.hpp"
#include "src/aruwlib/errors/create_errors.hpp"

using namespace aruwlib::motor;

namespace aruwsrc
{

namespace control
{
    TurretSubsystem::TurretSubsystem() :
        pitchMotor(PITCH_MOTOR_ID, CAN_BUS_MOTORS, true),
        yawMotor(YAW_MOTOR_ID, CAN_BUS_MOTORS, false),
        currPitchAngle(0.0f, 0.0f, 360.0f),
        currYawAngle(0.0f, 0.0f, 360.0f),
        yawTarget(TURRET_START_ANGLE, 0.0f, 360.0f),
        pitchTarget(TURRET_START_ANGLE, 0.0f, 360.0f),
        prevYawTarget(TURRET_START_ANGLE, 0.0f, 360.0f),
        pretPitchTarget(TURRET_START_ANGLE, 0.0f, 360.0f)
    {}

    float TurretSubsystem::getYawAngleFromCenter() const
    {
        aruwlib::algorithms::ContiguousFloat yawAngleFromCenter(
            currYawAngle.getValue() - TURRET_START_ANGLE, -180.0f, 180.0f);
        return yawAngleFromCenter.getValue();
    }

    float TurretSubsystem::getPitchAngleFromCenter() const
    {
        aruwlib::algorithms::ContiguousFloat yawAngleFromCenter(
            currPitchAngle.getValue() - TURRET_START_ANGLE, -180.0f, 180.0f);
        return yawAngleFromCenter.getValue();
    }

    int32_t TurretSubsystem::getYawVelocity() const
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

    int32_t TurretSubsystem::getPitchVelocity() const
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
    int32_t TurretSubsystem::getVelocity(const DjiMotor &motor) const
    {
        return 360 / 60 * motor.getShaftRPM();
    }

    bool TurretSubsystem::isTurretOnline() const
    {
        return pitchMotor.isMotorOnline() && yawMotor.isMotorOnline();
    }

    void TurretSubsystem::refresh()
    {
        updateCurrentTurretAngles();
    }

    void TurretSubsystem::updateCurrentTurretAngles()
    {
        updateCurrentYawAngle();
        updateCurrentPitchAngle();
    }

    void TurretSubsystem::updateCurrentYawAngle()
    {
        if (yawMotor.isMotorOnline())
        {
            currYawAngle.setValue(DjiMotor::encoderToDegrees(static_cast<uint16_t>(
                    yawMotor.encStore.getEncoderWrapped() - YAW_START_ENCODER_POSITION))
                    + TURRET_START_ANGLE);
        }
        else
        {
            currYawAngle.setValue(TURRET_START_ANGLE);
        }
    }

    void TurretSubsystem::updateCurrentPitchAngle()
    {
        if (pitchMotor.isMotorOnline())
        {
            currPitchAngle.setValue(DjiMotor::encoderToDegrees(static_cast<uint16_t>(
                   pitchMotor.encStore.getEncoderWrapped() - PITCH_START_ENCODER_POSITION))
                   + TURRET_START_ANGLE);
        }
        else
        {
            currPitchAngle.setValue(TURRET_START_ANGLE);
        }
    }

    void TurretSubsystem::setPitchMotorOutput(float out)
    {
        if (out > INT32_MAX || out < INT32_MIN)
        {
            RAISE_ERROR("pitch motor output invalid",
                    aruwlib::errors::TURRET, aruwlib::errors::INVALID_MOTOR_OUTPUT);
            return;
        }
        if (pitchMotor.isMotorOnline())
        {
            if ((getPitchAngleFromCenter() + TURRET_START_ANGLE >
                    TURRET_PITCH_MAX_ANGLE && out > 0) ||
                (getPitchAngleFromCenter() + TURRET_START_ANGLE <
                    TURRET_PITCH_MIN_ANGLE && out < 0))
            {
                pitchMotor.setDesiredOutput(0);
            }
            else
            {
                pitchMotor.setDesiredOutput(out);
            }
        }
    }

    void TurretSubsystem::setYawMotorOutput(float out)
    {
        if (out > INT32_MAX || out < INT32_MIN) {
            RAISE_ERROR("yaw motor output invalid",
                    aruwlib::errors::TURRET, aruwlib::errors::INVALID_MOTOR_OUTPUT);
            return;
        }
        if (yawMotor.isMotorOnline())
        {
            if ((getYawAngleFromCenter() + TURRET_START_ANGLE >
                    TURRET_YAW_MAX_ANGLE && out > 0) ||
                (getYawAngleFromCenter() + TURRET_START_ANGLE <
                    TURRET_YAW_MIN_ANGLE && out < 0))
            {
                yawMotor.setDesiredOutput(0);
            }
            else
            {
                yawMotor.setDesiredOutput(out);
            }
        }
    }

    const aruwlib::algorithms::ContiguousFloat& TurretSubsystem::getYawAngle()
    {
        updateCurrentYawAngle();
        return currYawAngle;
    }

    const aruwlib::algorithms::ContiguousFloat& TurretSubsystem::getPitchAngle()
    {
        updateCurrentPitchAngle();
        return currPitchAngle;
    }

    void TurretSubsystem::updatePrevYawTarget(const float& yaw)
    {
        prevYawTarget.setValue(yaw);
    }

    void TurretSubsystem::updatePrevPitchTarget(const float& pitch)
    {
        pretPitchTarget.setValue(pitch);
    }

    const aruwlib::algorithms::ContiguousFloat& TurretSubsystem::getPrevYawTarget() const
    {
        return prevYawTarget;
    }

    const aruwlib::algorithms::ContiguousFloat& TurretSubsystem::getPrevPitchTarget() const
    {
        return pretPitchTarget;
    }

    const aruwlib::algorithms::ContiguousFloat& TurretSubsystem::getYawTarget() const
    {
        return yawTarget;
    }

    const aruwlib::algorithms::ContiguousFloat& TurretSubsystem::getPitchTarget() const
    {
        return pitchTarget;
    }

    void TurretSubsystem::setYawTarget(float target)
    {
        yawTarget.setValue(target);
        yawTarget.limitValue(TURRET_YAW_MIN_ANGLE, TURRET_YAW_MAX_ANGLE);
    }

    void TurretSubsystem::setPitchTarget(float target)
    {
        pitchTarget.setValue(target);
        pitchTarget.limitValue(TURRET_PITCH_MIN_ANGLE, TURRET_PITCH_MAX_ANGLE);
    }

    void TurretSubsystem::yawFeedForwardCalculation(float desiredChassisRotation)
    {
        float chassisRotationProportional = FEED_FORWARD_KP
                * desiredChassisRotation
                * (fabsf(FEED_FORWARD_SIN_GAIN * sin(turretSubsystem->getYawAngleFromCenter()
                * aruwlib::algorithms::PI / 180.0f)) + 1.0f);

        chassisRotationDerivative = aruwlib::algorithms::lowPassFilter(chassisRotationDerivative,
                chassisSubsystem->getChassisDesiredRotation() - prevChassisRotationDesired,
                FEED_FORWARD_DERIVATIVE_LOW_PASS);

        float chassisRotationFeedForward = aruwlib::algorithms::limitVal<float>(
                chassisRotationProportional + FEED_FORWARD_KD * chassisRotationDerivative,
                -FEED_FORWARD_MAX_OUTPUT, FEED_FORWARD_MAX_OUTPUT);


    }

}  // namespace control

}  // namespace aruwsrc
