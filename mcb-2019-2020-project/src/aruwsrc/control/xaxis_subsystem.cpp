#include "xaxis_subsystem.hpp"
#include "src/aruwlib/algorithms/math_user_utils.hpp"

namespace aruwsrc
{

namespace control
{

    void XAxisSubsystem::setPosition(Position p) {
        xAxisPosition = p; 
        switch(xAxisPosition) {
            case Position::MIN_DISTANCE:
                xAxisRamp.setTarget(MIN_DISTANCE); 
                break;
            case Position::CENTER_DISTANCE:
                xAxisRamp.setTarget(CENTER_DISTANCE); 
                break;
            case Position::MAX_DISTANCE:
                xAxisRamp.setTarget(MAX_DISTANCE);
                break; 
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
        currentPosition = ramp->getValue(); 
    }

    float XAxisSubsystem::getPosition() const
    {
        return xAxisMotor.encStore.getEncoderUnwrapped() * (2 * aruwlib::algorithms::PI * X_AXIS_PULLEY_RADIUS / static_cast<float>(GM_3510_GEAR_RATIO));
    }

}

}