#include "agitator-subsystem.hpp"
#include "src/algorithms/math_user_utils.hpp"
#include <modm/math/filter/pid.hpp>
#include "src/control/subsystem.hpp"
#include "src/motor/dji_motor.hpp"

using namespace aruwlib::motor;

namespace aruwsrc
{

namespace control
{
    AgitatorSubsystem::AgitatorSubsystem(uint16_t gearRatio) :
        agitatorPositionPid(PID_P, PID_I, PID_D, PID_MAX_ERR_SUM, PID_MAX_OUT),
        agitatorMotor(AGITATOR_MOTOR_ID, AGITATOR_MOTOR_CAN_BUS),
        agitatorGearRatio(gearRatio),
        desiredAgitatorAngle(0)
    {}

    void AgitatorSubsystem::refresh()
    {
        agitatorRunPositionPid();
    }

    void AgitatorSubsystem::agitatorRunPositionPid()
    {
        agitatorPositionPid.update(agitatorEncoderToPosition() - desiredAgitatorAngle);
        agitatorMotor.setDesiredOutput(agitatorPositionPid.getValue());
    }

    float AgitatorSubsystem::agitatorEncoderToPosition()
    {
        return (2 * PI / ENC_RESOLUTION) * agitatorMotor.encStore.getEncoderUnwrapped() /
            static_cast<float>(agitatorGearRatio);
    }

    void AgitatorSubsystem::setAgitatorAngle(float newAngle)
    {
        desiredAgitatorAngle = newAngle;
    }

}  // namespace control

}  // namespace aruwsrc
