#include "agitator_subsystem.hpp"
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
        desiredAgitatorAngle(0.0f),
        agitatorCalibrationAngle(0.0f),
        agitatorIsCalibrated(false)
    {}

    void AgitatorSubsystem::refresh()
    {
        agitatorRunPositionPid();
    }

    void AgitatorSubsystem::agitatorRunPositionPid()
    {
        if (!agitatorIsCalibrated)
        {
            return;
        }
        agitatorPositionPid.update(agitatorEncoderToPosition() - desiredAgitatorAngle);
        agitatorMotor.setDesiredOutput(agitatorPositionPid.getValue());
    }

    void AgitatorSubsystem::agitatorCalibrateHere()
    {
        if (!agitatorMotor.isMotorOnline())
        {
            return;
        }
        agitatorCalibrationAngle = agitatorEncoderToPosition();
        agitatorIsCalibrated = true;
    }

    float AgitatorSubsystem::agitatorEncoderToPosition() const
    {
        if (agitatorIsCalibrated)
        {
            return 0.0f;
        }
        return (2 * PI / ENC_RESOLUTION) * agitatorMotor.encStore.getEncoderUnwrapped() /
            static_cast<float>(agitatorGearRatio) - agitatorCalibrationAngle;
    }

    void AgitatorSubsystem::setAgitatorAngle(float newAngle)
    {
        desiredAgitatorAngle = newAngle;
    }

    float AgitatorSubsystem::getAgitatorDesiredAngle(void) const
    {
        return desiredAgitatorAngle;
    }

}  // namespace control

}  // namespace aruwsrc
