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
    static constexpr aruwlib::motor::MotorId YAXIS_MOTOR_ID = aruwlib::motor::MOTOR8;
    static constexpr aruwlib::can::CanBus CAN_BUS_MOTORS = aruwlib::can::CanBus::CAN_BUS1;

    /**
     * Used in the `setPosition` function to set the desired position. Rather than
     * passing a raw position in cm, we define some valid positions that can be set.
     */
    enum class Position
    {
        MIN_DISTANCE,
        CENTER_DISTANCE,
        MAX_DISTANCE,
        INTERMEDIATE_DISTANCE,  ///< If `stop` is called this becomes the position.
    };

    YAxisSubsystem(aruwlib::Drivers *drivers, aruwlib::gpio::Digital::InputPin limitSwitchInitPin);

    /**
     * There are only a limited number of positions that the y axis mechanism
     * should be set, so through this abstraction we insure only these positions
     * are set.
     */
    mockable void setPosition(Position p);

    /**
     * Sets the position of the motor to be the current position and sets
     * the current position to be `INTERMEDIATE_DISTANCE`.
     */
    mockable void stop();

    /**
     * Initializes the YAxis motor if it is not yet initialized.
     * If the motor is initialized, the desired position ramp is updated and the
     * position PID controller is run.
     */
    void refresh() override;

    /**
     * @return The desired Position (an enum value, one of the descrete positions that
     *      may be set, **not** a measured value in cm).
     */
    mockable_inline Position getCurrDesiredPosition() const { return yAxisPosition; }

    /**
     * @return The last computed current position (which is computed on `refresh`), in cm.
     */
    mockable_inline float getCurrentPosition() const { return currentPosition; }

    /**
     * @return The motor's target position, in cm.
     */
    mockable_inline float getDesiredPosition() const { return yAxisRamp.getTarget(); }

    /**
     * Call to start the initialization sequence again.
     */
    mockable_inline void reInitialize() { initialized = false; }

    mockable_inline bool isInitialized() const { return initialized; }

private:
    // TODO(matthew) these PID parameters must be tuned with an actual mechanism
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

    /**
     * The proportional term used for the RPM PID controller used in initialization.
     */
    static constexpr float RPM_PID_P = 10.0f;

    /**
     * The target RPM for the PID controller involved in initializing the motor.
     */
    static constexpr float DESIRED_INIT_RPM = 1000.0f;

    aruwlib::Drivers *drivers;

    Position yAxisPosition;

    int64_t startEncoder;
    bool initialized;

    aruwlib::motor::DjiMotor yAxisMotor;
    algorithms::TurretPid yAxisPositionPid;
    aruwlib::algorithms::Ramp yAxisRamp;

    float currentPosition;
    uint32_t oldRampTime;

    /**
     * The limit switch used for initializing the subsystem.
     */
    aruwlib::gpio::Digital::InputPin limitSwitchInitPin;

    /**
     * Uses a proportional RPM controller to push the mechanism to the left
     * until a limit switch has been triggered, in which case the y axis motor
     * has been initialized.
     *
     * @note As this function runs a controller and sets the desired output
     *      of motors, it is not to be used at the same time as other control
     *      code (i.e. when the position controller is running).
     */
    void initializeYAxis();

    float getPosition() const;
};  // class YAxisSubsystem
}  // namespace engineer
}  // namespace aruwsrc
#endif  // YAXIS_SUBSYSTEM_HPP_
