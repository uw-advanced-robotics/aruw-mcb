#include "yaxis_subsystem.hpp"

#include <aruwlib/Drivers.hpp>
#include <aruwlib/algorithms/math_user_utils.hpp>
#include <aruwlib/architecture/clock.hpp>

namespace aruwsrc
{
namespace engineer
{
YAxisSubsystem::YAxisSubsystem(
    aruwlib::Drivers *drivers,
    aruwlib::gpio::Digital::InputPin limitSwitchInitPin)
    : aruwlib::control::Subsystem(drivers),
      drivers(drivers),
      yAxisPosition(Position::MIN_DISTANCE),
      startEncoder(0),
      initialized(false),
      yAxisMotor(drivers, YAXIS_MOTOR_ID, CAN_BUS_MOTORS, true, "yaxis motor"),
      yAxisPositionPid(
          PID_P,
          PID_I,
          PID_D,
          PID_MAX_ERROR_SUM,
          PID_MAX_OUTPUT,
          1.0f,
          0.0f,
          1.0f,
          0.0f),
      yAxisRamp(0.0f),
      currentPosition(0.0f),
      oldRampTime(0),
      limitSwitchInitPin(limitSwitchInitPin)
{
}

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
        case Position::INTERMEDIATE_DISTANCE:
            yAxisRamp.setTarget(currentPosition);
            break;
        default:
            break;
    }
}

void YAxisSubsystem::stop() { setPosition(Position::INTERMEDIATE_DISTANCE); }

void YAxisSubsystem::refresh()
{
    if (!initialized || !yAxisMotor.isMotorOnline())
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
        initialized = false;
        yAxisMotor.setDesiredOutput(0);
    }
    else
    {
        if (drivers->digital.read(limitSwitchInitPin))
        {
            // The limit switch has been triggered, meaning the initialization is complete.
            startEncoder = yAxisMotor.getEncoderUnwrapped();
            initialized = true;
            yAxisMotor.setDesiredOutput(0);
        }
        else
        {
            // The limit switch has not been triggered, super simple proportional RPM control.
            yAxisMotor.setDesiredOutput(RPM_PID_P * (DESIRED_INIT_RPM - yAxisMotor.getShaftRPM()));
        }
    }
}

float YAxisSubsystem::getPosition() const
{
    return ((yAxisMotor.getEncoderUnwrapped() - startEncoder) /
            static_cast<float>(aruwlib::motor::DjiMotor::ENC_RESOLUTION)) *
           (2.0f * aruwlib::algorithms::PI * Y_AXIS_PULLEY_RADIUS / GM_3510_GEAR_RATIO);
}
}  // namespace engineer
}  // namespace aruwsrc
