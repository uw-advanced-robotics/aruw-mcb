#ifndef YAXIS_SUBSYSTEM_HPP_
#define YAXIS_SUBSYSTEM_HPP_

#include <aruwlib/algorithms/ramp.hpp>
#include <aruwlib/communication/gpio/digital.hpp>
#include <aruwlib/control/subsystem.hpp>
#include <aruwlib/motor/dji_motor.hpp>
#include <modm/math/filter/ramp.hpp>

#include "aruwsrc/algorithms/turret_pid.hpp"

namespace aruwsrc
{
namespace engineer
{
/**
 * Controls the position of the Y-axis mechanism. Controlled by a belt mechansim
 * connected to a GM3510. This subsystem contains logic for initializing the motor
 * using a limit switch located on one side of the mechanism. This class also
 * contains a position controller in conjunction with ramping used once the motor
 * is initialized.
 */
class YAxisSubsystem : public aruwlib::control::Subsystem
{
public:
    /**
     * Used in the `setPosition` function to set the desired position. Rather than
     * passing a raw position, we define some valid positions that can be set.
     */
    enum class Position
    {
        MIN_DISTANCE,
        CENTER_DISTANCE,
        MAX_DISTANCE,
    };

    YAxisSubsystem(aruwlib::gpio::Digital::InputPin limitSwitchInitPin)
        : yAxisPosition(Position::MIN_DISTANCE),
          startEncoder(0),
          initialized(false),
          yAxisMotor(YAXIS_MOTOR_ID, CAN_BUS_MOTORS, true, "yaxis motor"),
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
          desiredPosition(0.0f),
          oldRampTime(0)
    {
    }

    void setPosition(Position p);

    /**
     * Uses a proportional RPM controller to push the mechanism to the left
     * until a limit switch has been triggered, in which case the y axis motor
     * has been initialized.
     */
    void initializeYAxis();

    /**
     * Initializes the YAxis motor if it is not yet initialized.
     * If the motor is initialized, the desired position ramp is updated and the
     * position PID controller is run.
     */
    void refresh();

    /**
     * @return The last compouted current position (which is computed on `refresh`).
     */
    float getCurrentPosition() const { return currentPosition; }

    /**
     * @return The motor's target position.
     */
    float getDesiredPosition() const { return yAxisRamp.getTarget(); }

    /**
     * Call to start the initialization sequence again.
     */
    void reInitialize() { initialized = false; }

    bool isInitialized() const { return initialized; }

private:
    static constexpr aruwlib::motor::MotorId YAXIS_MOTOR_ID = aruwlib::motor::MOTOR8;
    static constexpr aruwlib::can::CanBus CAN_BUS_MOTORS = aruwlib::can::CanBus::CAN_BUS1;

    static constexpr float PID_P = 100000.0f;
    static constexpr float PID_I = 0.0f;
    static constexpr float PID_D = 1000000.0f;
    static constexpr float PID_MAX_ERROR_SUM = 0.0f;
    static constexpr float PID_MAX_OUTPUT = 16000.0f;
    static constexpr float RAMP_SPEED = 1.0f;

    // units below are in centimeter (cm)
    static constexpr float MIN_DIST = 0.0f;
    static constexpr float CENTER_DIST = 15.0f;
    static constexpr float MAX_DIST = 30.0f;
    static constexpr float Y_AXIS_PULLEY_RADIUS = 2.5f;

    // 19:1 gear ratio
    static constexpr float GM_3510_GEAR_RATIO = 19.0f;

    ///< The proportional term used for the RPM PID controller used in initialization.
    static constexpr float RPM_PID_P = 10.0f;

    ///< The target RPM for the PID controller involved in initializing the motor.
    static constexpr float DESIRED_INIT_RPM = 1000.0f;

    Position yAxisPosition;

    int64_t startEncoder;
    bool initialized;

    aruwlib::motor::DjiMotor yAxisMotor;
    algorithms::TurretPid yAxisPositionPid;
    aruwlib::algorithms::Ramp yAxisRamp;

    float currentPosition;
    float desiredPosition;
    uint32_t oldRampTime;

    ///< The limit switch used for initializing the subsystem.
    aruwlib::gpio::Digital::InputPin limitSwitchInitPin;

    void updateMotorDisplacement(
        aruwlib::motor::DjiMotor* const motor,
        modm::filter::Ramp<float>* ramp);

    float getPosition() const;
};  // class YAxisSubsystem
}  // namespace engineer
}  // namespace aruwsrc
#endif  // YAXIS_SUBSYSTEM_HPP_
