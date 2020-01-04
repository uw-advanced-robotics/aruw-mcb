#include "agitator_subsystem.hpp"
#include "src/aruwlib/algorithms/math_user_utils.hpp"
#include <modm/math/filter/pid.hpp>
#include "src/aruwlib/control/subsystem.hpp"
#include "src/aruwlib/motor/dji_motor.hpp"

using namespace aruwlib::motor;

namespace aruwsrc
{

namespace control
{
    const int AgitatorSubsystem::DEFAULT_AGITATOR_JAMMED_TIMEOUT_PERIOD = 50;

    AgitatorSubsystem::AgitatorSubsystem(uint16_t gearRatio, int agitatorJamTimeout) :
        agitatorPositionPid(PID_P, PID_I, PID_D, PID_MAX_ERR_SUM, PID_MAX_OUT),
        agitatorMotor(AGITATOR_MOTOR_ID, AGITATOR_MOTOR_CAN_BUS),
        desiredAgitatorAngle(0.0f),
        agitatorCalibrationAngle(0.0f),
        agitatorIsCalibrated(false),
        agitatorJammedTimeout(agitatorJamTimeout),
        agitatorJammedTimeoutPeriod(agitatorJamTimeout),
        agitatorGearRatio(gearRatio)
    {
        agitatorJammedTimeout.stop();
    }

    void AgitatorSubsystem::armAgitatorUnjamTimer()
    {
        agitatorJammedTimeout.restart(agitatorJammedTimeoutPeriod);
    }

    void AgitatorSubsystem::disarmAgitatorUnjamTimer()
    {
        agitatorJammedTimeout.stop();
    }

    bool AgitatorSubsystem::isAgitatorJammed()
    {
        return agitatorJammedTimeout.isExpired();
    }

    void AgitatorSubsystem::refresh()
    {
        agitatorRunPositionPid();
    }

    void AgitatorSubsystem::agitatorRunPositionPid()
    {
        if (!agitatorIsCalibrated)
        {
            agitatorPositionPid.reset();
            return;
        }
        agitatorPositionPid.update(desiredAgitatorAngle - getAgitatorEncoderToPosition());
        agitatorMotor.setDesiredOutput(agitatorPositionPid.getValue());
    }

    bool AgitatorSubsystem::agitatorCalibrateHere()
    {
        if (!agitatorMotor.isMotorOnline())
        {
            return false;
        }
        agitatorCalibrationAngle = (2.0f * aruwlib::algorithms::PI / static_cast<float>(ENC_RESOLUTION)) *
            agitatorMotor.encStore.getEncoderUnwrapped() / agitatorGearRatio;
        agitatorIsCalibrated = true;
        return true;
    }

    float AgitatorSubsystem::getAgitatorEncoderToPosition() const
    {
        if (!agitatorIsCalibrated)
        {
            return 0.0f;
        }
        return (2.0f * aruwlib::algorithms::PI / static_cast<float>(ENC_RESOLUTION)) * agitatorMotor.encStore.getEncoderUnwrapped()
            / agitatorGearRatio - agitatorCalibrationAngle;
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
