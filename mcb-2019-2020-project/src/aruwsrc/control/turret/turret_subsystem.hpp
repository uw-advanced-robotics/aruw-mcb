#ifndef __TURRET_SUBSYSTEM_HPP__
#define __TURRET_SUBSYSTEM_HPP__

#include <aruwlib/algorithms/contiguous_float.hpp>
#include <aruwlib/algorithms/linear_interpolation.hpp>
#include <aruwlib/architecture/clock.hpp>
#include <aruwlib/control/subsystem.hpp>
#include <aruwlib/errors/create_errors.hpp>
#include <aruwlib/motor/dji_motor.hpp>
#include <modm/math/filter/pid.hpp>

namespace aruwsrc
{
namespace turret
{
template <typename Drivers>
class TurretSubsystem : public aruwlib::control::Subsystem<Drivers>
{
public:
    static constexpr float TURRET_START_ANGLE = 90.0f;
    static constexpr float TURRET_YAW_MIN_ANGLE = TURRET_START_ANGLE - 90.0f;
    static constexpr float TURRET_YAW_MAX_ANGLE = TURRET_START_ANGLE + 90.0f;
    static constexpr float TURRET_PITCH_MIN_ANGLE = TURRET_START_ANGLE - 13.0f;
    static constexpr float TURRET_PITCH_MAX_ANGLE = TURRET_START_ANGLE + 20.0f;

    TurretSubsystem()
        : pitchMotor(PITCH_MOTOR_ID, CAN_BUS_MOTORS, true, "pitch motor"),
          yawMotor(YAW_MOTOR_ID, CAN_BUS_MOTORS, false, "yaw motor"),
          currPitchAngle(0.0f, 0.0f, 360.0f),
          currYawAngle(0.0f, 0.0f, 360.0f),
          yawTarget(TURRET_START_ANGLE, 0.0f, 360.0f),
          pitchTarget(TURRET_START_ANGLE, 0.0f, 360.0f)
    {
    }

    void refresh() override { updateCurrentTurretAngles(); }

    bool isTurretOnline() const { return pitchMotor.isMotorOnline() && yawMotor.isMotorOnline(); }

    int32_t getYawVelocity() const
    {
        if (!yawMotor.isMotorOnline())
        {
            RAISE_ERROR(
                "trying to get velocity and yaw motor offline",
                aruwlib::errors::TURRET,
                aruwlib::errors::MOTOR_OFFLINE);
            // throw error
            return 0;
        }

        return getVelocity(yawMotor);
    }
    int32_t getPitchVelocity() const
    {
        if (!pitchMotor.isMotorOnline())
        {
            RAISE_ERROR(
                "trying to get velocity and pitch motor offline",
                aruwlib::errors::TURRET,
                aruwlib::errors::MOTOR_OFFLINE);
            return 0;
        }

        return getVelocity(pitchMotor);
    }

    float getYawAngleFromCenter() const
    {
        aruwlib::algorithms::ContiguousFloat yawAngleFromCenter(
            currYawAngle.getValue() - TURRET_START_ANGLE,
            -180.0f,
            180.0f);
        return yawAngleFromCenter.getValue();
    }

    float getPitchAngleFromCenter() const
    {
        aruwlib::algorithms::ContiguousFloat yawAngleFromCenter(
            currPitchAngle.getValue() - TURRET_START_ANGLE,
            -180.0f,
            180.0f);
        return yawAngleFromCenter.getValue();
    }

    const aruwlib::algorithms::ContiguousFloat& getYawAngle() const { return currYawAngle; }
    const aruwlib::algorithms::ContiguousFloat& getPitchAngle() const { return currPitchAngle; }

    void setYawMotorOutput(float out)
    {
        if (out > INT32_MAX || out < INT32_MIN)
        {
            RAISE_ERROR(
                "yaw motor output invalid",
                aruwlib::errors::TURRET,
                aruwlib::errors::INVALID_MOTOR_OUTPUT);
            return;
        }
        if (yawMotor.isMotorOnline())
        {
            if ((getYawAngleFromCenter() + TURRET_START_ANGLE > TURRET_YAW_MAX_ANGLE && out > 0) ||
                (getYawAngleFromCenter() + TURRET_START_ANGLE < TURRET_YAW_MIN_ANGLE && out < 0))
            {
                yawMotor.setDesiredOutput(0);
            }
            else
            {
                yawMotor.setDesiredOutput(out);
            }
        }
    }
    void setPitchMotorOutput(float out)
    {
        if (out > INT32_MAX || out < INT32_MIN)
        {
            RAISE_ERROR(
                "pitch motor output invalid",
                aruwlib::errors::TURRET,
                aruwlib::errors::INVALID_MOTOR_OUTPUT);
            return;
        }
        if (pitchMotor.isMotorOnline())
        {
            if ((getPitchAngleFromCenter() + TURRET_START_ANGLE > TURRET_PITCH_MAX_ANGLE &&
                 out > 0) ||
                (getPitchAngleFromCenter() + TURRET_START_ANGLE < TURRET_PITCH_MIN_ANGLE &&
                 out < 0))
            {
                pitchMotor.setDesiredOutput(0);
            }
            else
            {
                pitchMotor.setDesiredOutput(out);
            }
        }
    }

    /**
     * Calculates a yaw output that uses the desired chassis rotation as a feed forward gain.
     *
     * The chassis rotation is given in desired wheel rpm.
     */
    float yawFeedForwardCalculation(float desiredChassisRotation)
    {
        // calculate feed forward
        float chassisRotationProportional =
            FEED_FORWARD_KP * desiredChassisRotation *
            (fabsf(
                 FEED_FORWARD_SIN_GAIN *
                 sinf(getYawAngleFromCenter() * aruwlib::algorithms::PI / 180.0f)) +
             1.0f);

        if (Drivers::remote.getUpdateCounter() != prevUpdateCounterChassisRotateDerivative)
        {
            chassisRotateDerivativeInterpolation.update(
                desiredChassisRotation - feedforwardPrevChassisRotationDesired);
        }
        prevUpdateCounterChassisRotateDerivative = Drivers::remote.getUpdateCounter();
        float derivativeInterpolated = chassisRotateDerivativeInterpolation.getInterpolatedValue(
            aruwlib::arch::clock::getTimeMilliseconds());

        feedforwardChassisRotateDerivative = aruwlib::algorithms::lowPassFilter(
            feedforwardChassisRotateDerivative,
            derivativeInterpolated,
            FEED_FORWARD_DERIVATIVE_LOW_PASS);

        float chassisRotationFeedForward = aruwlib::algorithms::limitVal<float>(
            chassisRotationProportional + FEED_FORWARD_KD * feedforwardChassisRotateDerivative,
            -FEED_FORWARD_MAX_OUTPUT,
            FEED_FORWARD_MAX_OUTPUT);

        feedforwardPrevChassisRotationDesired = desiredChassisRotation;

        if ((chassisRotationFeedForward > 0.0f &&
             getYawAngle().getValue() > TurretSubsystem::TURRET_YAW_MAX_ANGLE) ||
            (chassisRotationFeedForward < 0.0f &&
             getYawAngle().getValue() < TurretSubsystem::TURRET_YAW_MIN_ANGLE))
        {
            chassisRotationFeedForward = 0.0f;
        }
        return chassisRotationFeedForward;
    }

    /**
     * Set a target angle in chassis frame, the angle is accordingly limited.
     * Note that since there is no controller in this subsystem, this target
     * angle merely acts as a safe way to set angles when using a position controller.
     */
    void setYawTarget(float target)
    {
        yawTarget.setValue(target);
        yawTarget.setValue(aruwlib::algorithms::ContiguousFloat::limitValue(
            yawTarget,
            TURRET_YAW_MIN_ANGLE,
            TURRET_YAW_MAX_ANGLE));
    }
    void setPitchTarget(float target)
    {
        pitchTarget.setValue(target);
        pitchTarget.setValue(aruwlib::algorithms::ContiguousFloat::limitValue(
            pitchTarget,
            TURRET_PITCH_MIN_ANGLE,
            TURRET_PITCH_MAX_ANGLE));
    }

    float getYawTarget() const { return yawTarget.getValue(); }
    float getPitchTarget() const { return pitchTarget.getValue(); }

    void updateCurrentTurretAngles()
    {
        updateCurrentYawAngle();
        updateCurrentPitchAngle();
    }

private:
    const uint16_t YAW_START_ENCODER_POSITION = 8160;
    const uint16_t PITCH_START_ENCODER_POSITION = 4100;

    static constexpr float FEED_FORWARD_KP = 2.0f;
    static constexpr float FEED_FORWARD_SIN_GAIN = 2.0f;
    static constexpr float FEED_FORWARD_KD = 1.0f;
    static constexpr float FEED_FORWARD_MAX_OUTPUT = 20000.0f;
    static constexpr float FEED_FORWARD_DERIVATIVE_LOW_PASS = 0.8f;

    const aruwlib::can::CanBus CAN_BUS_MOTORS = aruwlib::can::CanBus::CAN_BUS1;
    static const aruwlib::motor::MotorId PITCH_MOTOR_ID = aruwlib::motor::MOTOR6;
    static const aruwlib::motor::MotorId YAW_MOTOR_ID = aruwlib::motor::MOTOR5;

    uint32_t prevUpdateCounterChassisRotateDerivative = 0;
    aruwlib::algorithms::LinearInterpolation chassisRotateDerivativeInterpolation;
    float feedforwardChassisRotateDerivative = 0.0f;
    float feedforwardPrevChassisRotationDesired = 0.0f;

    aruwlib::motor::DjiMotor<Drivers> pitchMotor;
    aruwlib::motor::DjiMotor<Drivers> yawMotor;

    aruwlib::algorithms::ContiguousFloat currPitchAngle;
    aruwlib::algorithms::ContiguousFloat currYawAngle;

    aruwlib::algorithms::ContiguousFloat yawTarget;
    aruwlib::algorithms::ContiguousFloat pitchTarget;

    void updateCurrentYawAngle()
    {
        if (yawMotor.isMotorOnline())
        {
            currYawAngle.setValue(
                aruwlib::motor::DjiMotor<Drivers>::encoderToDegrees(static_cast<uint16_t>(
                    yawMotor.encStore.getEncoderWrapped() - YAW_START_ENCODER_POSITION)) +
                TURRET_START_ANGLE);
        }
        else
        {
            currYawAngle.setValue(TURRET_START_ANGLE);
        }
    }
    void updateCurrentPitchAngle()
    {
        if (pitchMotor.isMotorOnline())
        {
            currPitchAngle.setValue(
                aruwlib::motor::DjiMotor<Drivers>::encoderToDegrees(static_cast<uint16_t>(
                    pitchMotor.encStore.getEncoderWrapped() - PITCH_START_ENCODER_POSITION)) +
                TURRET_START_ANGLE);
        }
        else
        {
            currPitchAngle.setValue(TURRET_START_ANGLE);
        }
    }

    int32_t getVelocity(const aruwlib::motor::DjiMotor<Drivers>& motor) const
    {
        return 360 / 60 * motor.getShaftRPM();
    }
};

}  // namespace turret

}  // namespace aruwsrc

#endif
