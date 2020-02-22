#ifndef __RESERVOIR_42MM_SUBSYSTEM_HPP__
#define __RESERVOIR_42MM_SUBSYSTEM_HPP__

#include <modm/math/filter/pid.hpp>
#include "src/aruwlib/control/command_scheduler.hpp"
#include "src/aruwlib/control/subsystem.hpp"
#include "src/aruwlib/motor/dji_motor.hpp"
#include "src/aruwlib/algorithms/math_user_utils.hpp"

using namespace aruwlib::control;

namespace aruwsrc
{

namespace engineer
{

class Reservoir42mmSubsystem : public aruwlib::control::Subsystem
{
 public:
    Reservoir42mmSubsystem();

    void refresh();

    float getAngle() const;  

    float getDesiredAngle() const;

    void setDesiredAngle(float newAngle);

    bool reservoirCalibrateHere();

    void reservoirToggleState();

    bool isClosed();

 private:
    static constexpr float RESERVOIR_42MM_GEAR_RATIO = 36.0f;

    static const aruwlib::motor::MotorId RESERVOIR_42MM_MOTOR_ID = aruwlib::motor::MOTOR5;
    const aruwlib::can::CanBus CAN_BUS_MOTORS = aruwlib::can::CanBus::CAN_BUS1;

    aruwlib::motor::DjiMotor reservoirMotor;

    // PID values
    const float PID_P = 80000.0f; // TODO
    const float PID_I = 0.0f;
    const float PID_D = 800000.0f;
    const float PID_MAX_ERROR_SUM = 0.0f;
    const float PID_MAX_OUTPUT = 16000;

    modm::Pid<float> positionPid;

    float desiredReservoirAngle;

    float reservoirCalibratedAngle;

    bool reservoirIsCalibrated;

    bool reservoirIsClosed;

    void reservoirRunPositionPid(void);

    float getUncalibratedReservoirAngle(void) const;
};

}  // namespace engineer

}  // namespace aruwsrc

#endif  // __RESERVOIR_42MM_SUBSYSTEM_HPP__