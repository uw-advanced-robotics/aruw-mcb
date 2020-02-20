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
        currYawAngle(0.0f, 0.0f, 360.0f),
        currPitchAngle(0.0f, 0.0f, 360.0f)
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

    float TurretSubsystem::getYawVelocity() const {
        return getVelocity(yawMotor);
    }

    float TurretSubsystem::getPitchVelocity() const {
        return getVelocity(pitchMotor);
    }

    // units: degrees per second
    float TurretSubsystem::getVelocity(const DjiMotor &motor) const {
        return 6.0f * motor.getShaftRPM();
    }

    bool TurretSubsystem::isTurretOnline() const {
        return pitchMotor.isMotorOnline() && yawMotor.isMotorOnline();
    }

    void TurretSubsystem::refresh() {
        updateCurrentTurretAngles();
    }

    void TurretSubsystem::updateCurrentTurretAngles()
    {
        if (yawMotor.isMotorOnline())
        {
            currYawAngle.setValue(DjiMotor::encoderToDegrees(
                    yawMotor.encStore.getEncoderWrapped() - YAW_START_ENCODER_POSITION)
                    + TURRET_START_ANGLE);
        }
        else
        {
            currYawAngle.setValue(TURRET_START_ANGLE);
        }
        if (pitchMotor.isMotorOnline())
        {
            currPitchAngle.setValue(DjiMotor::encoderToDegrees(
                    pitchMotor.encStore.getEncoderWrapped() - PITCH_START_ENCODER_POSITION)
                    + TURRET_START_ANGLE);
        }
        else
        {
            currPitchAngle.setValue(TURRET_START_ANGLE);
        }
    }

    /// \todo fix this
    void TurretSubsystem::setPitchMotorOutput(float out) {
        if (out > INT32_MAX) {
            // return error
            return;
        }
        if (isTurretOnline()) {
            if ((getPitchAngleFromCenter() + TURRET_START_ANGLE > TURRET_PITCH_MAX_ANGLE && out > 0) ||
                (getPitchAngleFromCenter() + TURRET_START_ANGLE < TURRET_PITCH_MIN_ANGLE && out < 0)) {
                pitchMotor.setDesiredOutput(0);
            } else {
                pitchMotor.setDesiredOutput(out);
            }
        }
    }

    void TurretSubsystem::setYawMotorOutput(float out) {
        yawMotor.setDesiredOutput(out);
        // if (out > INT32_MAX) {
        //     // return error
        //     return;
        // }
        // if (isTurretOnline()) {
        //     if ((getYawAngleFromCenter() + TURRET_START_ANGLE > TURRET_YAW_MAX_ANGLE && out > 0) ||
        //         (getYawAngleFromCenter() + TURRET_START_ANGLE < TURRET_YAW_MIN_ANGLE && out < 0)) {
        //         yawMotor.setDesiredOutput(0);
        //     } else {
        //         yawMotor.setDesiredOutput(out);
        //     }
        // }
    }

    float TurretSubsystem::getRemoteXMovement() {
        return aruwlib::Remote::getChannel(Remote::Channel::RIGHT_HORIZONTAL) * REMOTE_INPUT_SCALER;
    }

    float TurretSubsystem::getRemoteYMovement() {
        return aruwlib::Remote::getChannel(Remote::Channel::RIGHT_VERTICAL) * REMOTE_INPUT_SCALER;
    }

    const aruwlib::algorithms::ContiguousFloat& TurretSubsystem::getYawAngle() const
    {
        return currYawAngle;
    }

    const aruwlib::algorithms::ContiguousFloat& TurretSubsystem::getPitchAngle() const
    {
        return currPitchAngle;
    }

}  // namespace control

}  // namespace aruwsrc
