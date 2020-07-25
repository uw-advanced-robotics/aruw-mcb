#include "xaxis_subsystem.hpp"
#include "src/aruwlib/algorithms/math_user_utils.hpp"

namespace aruwsrc
{
namespace engineer
{
    void XAxisSubsystem::setPosition(Position p) {
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

    void XAxisSubsystem::refresh(void) {
        if (!isInitialized) {
            initializeYAxis();
        }
        updateMotorDisplacement(
            &yAxisMotor,
            &yAxisRamp);

    }

    float error; 

    void XAxisSubsystem::updateMotorDisplacement(
        aruwlib::motor::DjiMotor* motor,
        modm::filter::Ramp<float>* ramp
    ) {

        ramp->update();
        error = ramp->getValue() - getPosition();
        yAxisPositionPid.update(error);
        motor->setDesiredOutput(yAxisPositionPid.getValue());
        currentPosition = ramp->getValue(); 
    }

    void XAxisSubsystem::initializeYAxis() {
        startEncoder = yAxisMotor.encStore.getEncoderUnwrapped();
        isInitialized = true; 
        
    }
    float XAxisSubsystem::getPosition() const
    {
        return ((yAxisMotor.encStore.getEncoderUnwrapped() - startEncoder) / 8192.0f) * (2 * aruwlib::algorithms::PI * Y_AXIS_PULLEY_RADIUS / static_cast<float>(GM_3510_GEAR_RATIO));
    }
}  // namespace engineer
}  // namespace control
