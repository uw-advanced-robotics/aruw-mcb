#ifndef __AGITATOR_SUBSYSTEM_HPP__
#define __AGITATOR_SUBSYSTEM_HPP__

#include <modm/math/filter/pid.hpp>
#include <modm/processing/timer/timeout.hpp>

#include "src/control/subsystem.hpp"
#include "src/motor/dji_motor.hpp"

namespace aruwsrc
{

namespace control
{

class AgitatorSubsystem : aruwlib::control::Subsystem {
 public:
    AgitatorSubsystem(uint16_t gearRatio,
       int agitatorJamTimeout = DEFAULT_AGITATOR_JAMMED_TIMEOUT_PERIOD);

    void refresh(void);

    void setAgitatorAngle(float newAngle);

    float getAgitatorEncoderToPosition(void) const;

    float getAgitatorDesiredAngle(void) const;

    void agitatorCalibrateHere(void);

    void armAgitatorUnjamTimer(void);

    void disarmAgitatorUnjamTimer(void);

    bool isAgitatorJammed(void);

 private:
    static const int DEFAULT_AGITATOR_JAMMED_TIMEOUT_PERIOD;
    const float PID_P = 70000.0f;
    const float PID_I = 0.0f;
    const float PID_D = 1000000.0f;
    const float PID_MAX_ERR_SUM = 0.0f;
    const float PID_MAX_OUT = 16000.0f;

    const aruwlib::motor::MotorId AGITATOR_MOTOR_ID = aruwlib::motor::MOTOR7;
    const aruwlib::can::CanBus AGITATOR_MOTOR_CAN_BUS = aruwlib::can::CanBus::CAN_BUS1;

    modm::Pid<float> agitatorPositionPid;

    aruwlib::motor::DjiMotor agitatorMotor;

    uint16_t agitatorGearRatio;

    float desiredAgitatorAngle;

    float agitatorCalibrationAngle;

    bool agitatorIsCalibrated;

    modm::ShortTimeout agitatorJammedTimeout;

    int agitatorJammedTimeoutPeriod;

    void agitatorRunPositionPid(void);
};

}  // namespace control

}  // namespace aruwsrc

#endif
