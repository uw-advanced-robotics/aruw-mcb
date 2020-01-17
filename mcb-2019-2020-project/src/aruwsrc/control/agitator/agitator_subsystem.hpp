#ifndef __AGITATOR_SUBSYSTEM_HPP__
#define __AGITATOR_SUBSYSTEM_HPP__

#include <modm/math/filter/pid.hpp>
#include <modm/processing/timer/timeout.hpp>

#include "src/aruwlib/control/subsystem.hpp"
#include "src/aruwlib/motor/dji_motor.hpp"

namespace aruwsrc
{

namespace agitator
{

class AgitatorSubsystem : public aruwlib::control::Subsystem {
 public:
    AgitatorSubsystem();

    void refresh();

    void setAgitatorAngle(float newAngle);

    float getAgitatorAngle() const;

    float getAgitatorDesiredAngle() const;

    bool agitatorCalibrateHere();

    void armAgitatorUnjamTimer(uint32_t predictedRotateTime);

    void disarmAgitatorUnjamTimer();

    bool isAgitatorJammed();

 private:
    // agitator gear ratio, for determining shaft rotation angle
    static constexpr float AGITATOR_GEAR_RATIO = 36.0f;

    // if no default agitator timeout period is specified for whatever reason, 50 ms is used
    static const int DEFAULT_AGITATOR_JAMMED_TIMEOUT_PERIOD = 50;

    // we add on this amount of "tolerance" to the predicted rotate time since some times it
    // takes longer than predicted and we only want to unjam when we are actually jammed
    // measured in ms
    static const int JAMMED_TOLERANCE_PERIOD = 10;

    // position pid terms
    const float PID_P = 190000.0f;
    const float PID_I = 0.0f;
    const float PID_D = 1500000.0f;
    const float PID_MAX_ERR_SUM = 0.0f;
    const float PID_MAX_OUT = 16000.0f;

    const aruwlib::motor::MotorId AGITATOR_MOTOR_ID = aruwlib::motor::MOTOR7;
    const aruwlib::can::CanBus AGITATOR_MOTOR_CAN_BUS = aruwlib::can::CanBus::CAN_BUS1;

    modm::Pid<float> agitatorPositionPid;

    aruwlib::motor::DjiMotor agitatorMotor;

    // The user desired angle, measured in radians.
    // The agitator uses unwrapped angle.
    float desiredAgitatorAngle;

    // You can calibrate the agitator, which will set the current agitator angle
    // to zero radians.
    float agitatorCalibratedZeroAngle;

    // Whether or not the agitator has been calibrated yet. You should calibrate the
    // agitator before using it.
    bool agitatorIsCalibrated;

    // A timeout that is used to determine whether or not the agitator is jammed.
    // If the agitator has not reached the desired position in a certain time, the
    // agitator is considered jammed.
    modm::ShortTimeout agitatorJammedTimeout;

    // the current agitator timeout time
    int agitatorJammedTimeoutPeriod;

    void agitatorRunPositionPid();

    float getUncalibratedAgitatorAngle() const;
};

}  // namespace control

}  // namespace aruwsrc

#endif
