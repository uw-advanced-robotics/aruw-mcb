#include "xaxis_subsystem.hpp"
#include "src/aruwlib/algorithms/math_user_utils.hpp"

namespace aruwsrc
{

namespace control
{
    const aruwlib::motor::MotorId XAxisSubsystem::XAXIS_MOTOR_ID = aruwlib::motor::MOTOR8;

    void XAxisSubsystem::setPosition(float desiredPosition) {
        if (desiredPosition >= MIN_DISTANCE && desiredPosition <= MAX_DISTANCE) {
            this->desiredPosition = desiredPosition;
            xAxisRamp.setTarget(this->desiredPosition);
        }
    }

    void XAxisSubsystem::refresh(void) {
        if (!xAxisRamp.isTargetReached()) {
            updateMotorDisplacement(
                &xAxisPositionPid,
                &xAxisMotor,
                &xAxisRamp);
        }
    }

    void XAxisSubsystem::updateMotorDisplacement(
        modm::Pid<float>* pid,
        aruwlib::motor::DjiMotor* motor,
        modm::filter::Ramp<float>* ramp
    ) {

        ramp->update();
        float error = ramp->getValue() - getPosition();
        xAxisPositionPid.update(error);
        motor->setDesiredOutput(xAxisPositionPid.getValue());
    }

    float XAxisSubsystem::getPosition()
    {
        return xAxisMotor.encStore.getEncoderUnwrapped() * (2 * aruwlib::algorithms::PI * X_AXIS_PULLEY_RADIUS / (float) GM_3510_GEAR_RATIO);
    }

}

}