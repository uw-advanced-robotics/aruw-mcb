#ifndef __SUBSYSTEM_SENTINEL_DRIVE_HPP__
#define __SUBSYSTEM_SENTINEL_DRIVE_HPP__

#include <aruwlib/communication/gpio/digital.hpp>
#include <aruwlib/control/command_scheduler.hpp>
#include <aruwlib/control/subsystem.hpp>
#include <aruwlib/errors/create_errors.hpp>
#include <aruwlib/motor/dji_motor.hpp>
#include <modm/math/filter/pid.hpp>

namespace aruwsrc
{
namespace control
{
template <typename Drivers> class SentinelDriveSubsystem : public aruwlib::control::Subsystem
{
public:
    static constexpr float MAX_POWER_CONSUMPTION = 30.0f;
    static constexpr float MAX_ENERGY_BUFFER = 200.0f;

    // length of the rail we own, in mm
    // the competition rail length is actually 4650mm
    static constexpr float RAIL_LENGTH = 1900;

    SentinelDriveSubsystem(
        aruwlib::motor::MotorId leftMotorId = LEFT_MOTOR_ID,
        aruwlib::motor::MotorId rightMotorId = RIGHT_MOTOR_ID)
        : leftWheel(leftMotorId, CAN_BUS_MOTORS, false, "left sentinel drive motor"),
          rightWheel(rightMotorId, CAN_BUS_MOTORS, false, "right sentinel drive motor"),
          velocityPidLeftWheel(PID_P, PID_I, PID_D, PID_MAX_ERROR_SUM, PID_MAX_OUTPUT),
          velocityPidRightWheel(PID_P, PID_I, PID_D, PID_MAX_ERROR_SUM, PID_MAX_OUTPUT),
          desiredRpm(0)
    {
    }

    void initialize() override
    {
        aruwlib::Drivers::digital.configureInputPullMode(
            leftLimitSwitch,
            aruwlib::gpio::Digital::InputPullMode::PullDown);
        aruwlib::Drivers::digital.configureInputPullMode(
            rightLimitSwitch,
            aruwlib::gpio::Digital::InputPullMode::PullDown);
    }

    /**
     * Returns absolute position of the sentinel, relative to the left end of the rail (when rail
     * is viewed from the front)
     */
    float absolutePosition()
    {
        float leftPosition = distanceFromEncoder(&leftWheel) - leftZeroRailOffset;
        float rightPosition = distanceFromEncoder(&rightWheel) - rightZeroRailOffset;
        float average = 0.0f;
        if (leftWheel.isMotorOnline() && rightWheel.isMotorOnline())
        {
            average = (leftPosition + rightPosition) / 2.0f;
        }
        else if (leftWheel.isMotorOnline())
        {
            RAISE_ERROR(
                "right sentinel drive motor offline",
                aruwlib::errors::Location::SUBSYSTEM,
                aruwlib::errors::ErrorType::MOTOR_OFFLINE);
            average = leftPosition;
        }
        else if (rightWheel.isMotorOnline())
        {
            RAISE_ERROR(
                "left sentinel drive motor offline",
                aruwlib::errors::Location::SUBSYSTEM,
                aruwlib::errors::ErrorType::MOTOR_OFFLINE);
            average = rightPosition;
        }
        else
        {
            RAISE_ERROR(
                "both sentinel drive motors offline",
                aruwlib::errors::Location::SUBSYSTEM,
                aruwlib::errors::ErrorType::MOTOR_OFFLINE);
            average = 0.0f;
        }
        return aruwlib::algorithms::limitVal<float>(
            average,
            0.0f,
            SentinelDriveSubsystem::RAIL_LENGTH);
    }

    void setDesiredRpm(float desRpm) { desiredRpm = desRpm; }

    void refresh() override
    {
        velocityPidLeftWheel.update(desiredRpm - leftWheel.getShaftRPM());
        leftWheel.setDesiredOutput(velocityPidLeftWheel.getValue());
        velocityPidRightWheel.update(desiredRpm - rightWheel.getShaftRPM());
        rightWheel.setDesiredOutput(velocityPidRightWheel.getValue());
        // constantly poll the limit switches, resetting offset if needed
        resetOffsetFromLimitSwitch();
    }

private:
    static constexpr aruwlib::motor::MotorId LEFT_MOTOR_ID = aruwlib::motor::MOTOR6;
    static constexpr aruwlib::motor::MotorId RIGHT_MOTOR_ID = aruwlib::motor::MOTOR5;
    const aruwlib::can::CanBus CAN_BUS_MOTORS = aruwlib::can::CanBus::CAN_BUS1;
    const aruwlib::gpio::Digital::InputPin leftLimitSwitch = aruwlib::gpio::Digital::InputPin::A;
    const aruwlib::gpio::Digital::InputPin rightLimitSwitch = aruwlib::gpio::Digital::InputPin::B;

    const float PID_P = 5.0f;
    const float PID_I = 0.0f;
    const float PID_D = 0.1f;
    const float PID_MAX_ERROR_SUM = 0.0f;
    const float PID_MAX_OUTPUT = 16000;

    // radius of the wheel in mm
    static constexpr float WHEEL_RADIUS = 35.0f;
    static constexpr float GEAR_RATIO = 19.0f;

    aruwlib::motor::DjiMotor leftWheel;

    aruwlib::motor::DjiMotor rightWheel;

    modm::Pid<float> velocityPidLeftWheel;

    modm::Pid<float> velocityPidRightWheel;

    float desiredRpm;
    float leftZeroRailOffset = 0;
    float rightZeroRailOffset = 0;

    // Resets the encoder offset used to determine position of the sentinel on the rail depending on
    // which limit switch is hit. If neither limit switch is hit, no-op. Left limit switch indicates
    // being at the start of the rail, right limit switch indicates end of rail.
    void resetOffsetFromLimitSwitch()
    {
        // DigitalPin where limit switch is placed
        if (aruwlib::Drivers::digital.read(leftLimitSwitch))
        {
            leftZeroRailOffset = distanceFromEncoder(&leftWheel);
            rightZeroRailOffset = distanceFromEncoder(&rightWheel);
        }
        else if (aruwlib::Drivers::digital.read(rightLimitSwitch))
        {
            leftZeroRailOffset = RAIL_LENGTH - distanceFromEncoder(&leftWheel);
            rightZeroRailOffset = RAIL_LENGTH - distanceFromEncoder(&rightWheel);
        }
    }

    // Returns the distance covered by the sentinel wheel on the rail
    // with respect to the encoders
    // Equation used: Arc Length = Angle * numberOfRotations * radius
    // Here we get the radius from the getEncoderUnwrapped function
    float distanceFromEncoder(aruwlib::motor::DjiMotor* motor)
    {
        float unwrappedAngle = motor->encStore.getEncoderUnwrapped();
        float numberOfRotations = unwrappedAngle / (aruwlib::motor::DjiMotor::ENC_RESOLUTION);
        return numberOfRotations * 2.0f * aruwlib::algorithms::PI * WHEEL_RADIUS / GEAR_RATIO;
    }
};

}  // namespace control

}  // namespace aruwsrc

#endif
