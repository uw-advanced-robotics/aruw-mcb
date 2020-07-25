#include "yaxis_subsystem.hpp"

#include <aruwlib/algorithms/math_user_utils.hpp>
#include <aruwlib/architecture/clock.hpp>

namespace aruwsrc
{
namespace engineer
{
void YAxisSubsystem::setPosition(Position p)
{
    yAxisPosition = p;
    switch (yAxisPosition)
    {
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

void YAxisSubsystem::refresh()
{
    if (!isInitialized)
    {
        initializeYAxis();
    }
    else
    {
        // Update the ramp.
        uint32_t newTime = aruwlib::arch::clock::getTimeMilliseconds();
        uint32_t dt = 2;  // Default if the dt is invalid.
        if (newTime > oldRampTime)
        {
            dt = newTime - oldRampTime;
        }
        oldRampTime = newTime;
        float increment = RAMP_SPEED * static_cast<float>(dt);
        yAxisRamp.update(increment);
        // Update the PID controller.
        currentPosition = getPosition();
        yAxisPositionPid.runController(
            yAxisRamp.getValue() - currentPosition,
            yAxisMotor.getShaftRPM());
    }
}

void YAxisSubsystem::initializeYAxis()
{
    if (!yAxisMotor.isMotorOnline())
    {
        isInitialized = false;
    }
    else
    {
        if (false)  // TODO(matthew)
        {
            // The limit switch has been triggered, meaning the initialization is complete
            startEncoder = yAxisMotor.encStore.getEncoderUnwrapped();
            isInitialized = true;
        }
        else
        {
            // The limit switch has not been triggered, open loop push to the left
            yAxisMotor.setDesiredOutput(5000);
        }
    }
}

float YAxisSubsystem::getPosition() const
{
    return ((yAxisMotor.encStore.getEncoderUnwrapped() - startEncoder) /
            static_cast<float>(aruwlib::motor::DjiMotor::ENC_RESOLUTION)) *
           (2.0f * aruwlib::algorithms::PI * Y_AXIS_PULLEY_RADIUS / GM_3510_GEAR_RATIO);
}
}  // namespace engineer
}  // namespace aruwsrc
