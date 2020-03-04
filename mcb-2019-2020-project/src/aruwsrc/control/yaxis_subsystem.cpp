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
        watchDistance = getPosition(); //
        // if (! online), false 
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

    void YAxisSubsystem::initializeYAxis() {
        startEncoder = yAxisMotor.encStore.getEncoderUnwrapped();
        
    }
    float YAxisSubsystem::getPosition() const
    {
        // yAxisMotor->setDesiredOutput(yAxisMotor.getOutputDesired - (yAxisMotor.encStore.getEncoderUnwrapped() / 8192.0f) * (2 * aruwlib::algorithms::PI * Y_AXIS_PULLEY_RADIUS / static_cast<float>(GM_3510_GEAR_RATIO)));
        return ((yAxisMotor.encStore.getEncoderUnwrapped() - startEncoder) / 8192.0f) * (2 * aruwlib::algorithms::PI * Y_AXIS_PULLEY_RADIUS / static_cast<float>(GM_3510_GEAR_RATIO));
    }

}

}