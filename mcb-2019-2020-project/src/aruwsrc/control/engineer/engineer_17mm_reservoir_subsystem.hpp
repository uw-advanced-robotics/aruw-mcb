#ifndef __ENGINEER_17MM_RESERVOIR_SUBSYSTEM_HPP__
#define __ENGINEER_17MM_RESERVOIR_SUBSYSTEM_HPP__

#include <modm/math/filter/pid.hpp>
#include "src/aruwlib/control/subsystem.hpp"
#include "src/aruwlib/motor/dji_motor.hpp"
#include "src/aruwlib/algorithms/math_user_utils.hpp"

using namespace aruwlib::control;

namespace aruwsrc
{

namespace control
{

class Engineer17mmReservoirSubsystem : public aruwlib::control::Subsystem
{
 public:
    Engineer17mmReservoirSubsystem();

    void refresh(void);

    void setReservoirAngle(float newAngle);

    float getReservoirAngle() const;

    float getReservoirDesiredAngle() const;

    bool reservoirCalibrateHere(void);
 private:
    static constexpr float RESERVOIR_GEAR_RATIO = 27.0f;

    // TODO: update all engineer motor assignments
    // might need another motor to dump the ammo
    static const aruwlib::motor::MotorId RESERVOIR_MOTOR_ID = aruwlib::motor::MOTOR1;
    const aruwlib::can::CanBus CAN_BUS_MOTORS = aruwlib::can::CanBus::CAN_BUS1;

    aruwlib::motor::DjiMotor reservoirMotor;

    // PID values
    const float PID_P = 10000.0f;
    const float PID_I = 0.0f;
    const float PID_D = 100000.0f;
    const float PID_MAX_ERROR_SUM = 0.0f;
    const float PID_MAX_OUTPUT = 16000;

    modm::Pid<float> reservoirPositionPid;

    float desiredReservoirAngle;

    float reservoirCalibratedAngle;

    bool reservoirIsCalibrated;

    void reservoirRunPositionPid(void);

    float getUncalibratedReservoirAngle(void) const;
};

}  // namespace control

}  // namespace aruwsrc

#endif  // __ENGINEER_17MM_RESERVOIR_SUBSYSTEM_HPP__