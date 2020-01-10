#ifndef __AGITATOR_SUBSYSTEM_HPP__
#define __AGITATOR_SUBSYSTEM_HPP__

#include <modm/math/filter/pid.hpp>
#include <modm/processing/timer/timeout.hpp>

#include "src/aruwlib/control/subsystem.hpp"
#include "src/aruwlib/motor/dji_motor.hpp"

namespace aruwsrc
{

namespace control
{

class AgitatorSubsystem : public aruwlib::control::Subsystem {
 public:
    AgitatorSubsystem(uint16_t gearRatio,
       int agitatorJamTimeout = DEFAULT_AGITATOR_JAMMED_TIMEOUT_PERIOD);

    void refresh(void);

    void setAgitatorAngle(float newAngle);

    float getAgitatorEncoderToPosition(void) const;

    float getAgitatorDesiredAngle(void) const;

    bool agitatorCalibrateHere(void);

    void armAgitatorUnjamTimer(uint32_t additionalUnjamTimeout);

    void disarmAgitatorUnjamTimer(void);

    bool isAgitatorJammed(void);

 private:
    static const int DEFAULT_AGITATOR_JAMMED_TIMEOUT_PERIOD;
    const float PID_P = 190000.0f;
    const float PID_I = 0.0f;
    const float PID_D = 1500000.0f;
    const float PID_MAX_ERR_SUM = 0.0f;
    const float PID_MAX_OUT = 16000.0f;

    const aruwlib::motor::MotorId AGITATOR_MOTOR_ID = aruwlib::motor::MOTOR7;
    const aruwlib::can::CanBus AGITATOR_MOTOR_CAN_BUS = aruwlib::can::CanBus::CAN_BUS1;

    modm::Pid<float> agitatorPositionPid;

    aruwlib::motor::DjiMotor agitatorMotor;

    float desiredAgitatorAngle;

    float agitatorCalibrationAngle;

    bool agitatorIsCalibrated;

    modm::ShortTimeout agitatorJammedTimeout;

    int agitatorJammedTimeoutPeriod;

    float agitatorGearRatio;

    void agitatorRunPositionPid(void);
};

}  // namespace control

}  // namespace aruwsrc

#endif
