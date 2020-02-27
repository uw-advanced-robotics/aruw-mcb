#include <algorithm>
#include <random>
#include <cfloat>
#include "turret_subsystem.hpp"
#include "src/aruwlib/communication/remote.hpp"
#include "src/aruwlib/algorithms/math_user_utils.hpp"
#include "src/aruwlib/control/controller_mapper.hpp"

using namespace aruwlib::motor;

namespace aruwsrc
{

namespace control
{
    TurretSubsystem::TurretSubsystem() :
        pitchMotor(PITCH_MOTOR_ID, CAN_BUS_MOTORS, true),
        yawMotor(YAW_MOTOR_ID, CAN_BUS_MOTORS, false),
        currPitchAngle(0.0f, 0.0f, 360.0f),
        currYawAngle(0.0f, 0.0f, 360.0f)
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
            // throw error
            return 0;
        }

        return getVelocity(yawMotor);
    }

    int32_t TurretSubsystem::getPitchVelocity() const
    {
        if (!pitchMotor.isMotorOnline())
        {
            // throw error
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
            // return error
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
            // return error
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

}  // namespace control

}  // namespace aruwsrc
