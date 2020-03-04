#include "yaxis_subsystem.hpp"
#include "src/aruwlib/algorithms/math_user_utils.hpp"

namespace aruwsrc
{

namespace control
{

    void YAxisSubsystem::setPosition(Position p) {
        yAxisPosition = p; 
        switch(yAxisPosition) {
            case Position::MIN_DISTANCE:
                yAxisRamp.setTarget(MIN_DIST); 
                break;
            case Position::CENTER_DISTANCE:
                yAxisRamp.setTarget(CENTER_DIST); 
                break;
            case Position::MAX_DISTANCE:
                yAxisRamp.setTarget(MAX_DIST);
                break; 
        }
    }

    void YAxisSubsystem::refresh(void) {
        if (!yAxisRamp.isTargetReached()) {
            updateMotorDisplacement(
                &yAxisMotor,
                &yAxisRamp);
        } 
    }

    void YAxisSubsystem::updateMotorDisplacement(
        aruwlib::motor::DjiMotor* motor,
        modm::filter::Ramp<float>* ramp
    ) {

        ramp->update();
        float error = ramp->getValue() - getPosition();
        yAxisPositionPid.update(error);
        motor->setDesiredOutput(yAxisPositionPid.getValue());
        currentPosition = ramp->getValue(); 
    }

    float YAxisSubsystem::getPosition() const
    {
        return yAxisMotor.encStore.getEncoderUnwrapped() * (2 * aruwlib::algorithms::PI * Y_AXIS_PULLEY_RADIUS / static_cast<float>(GM_3510_GEAR_RATIO));
    }

}

}