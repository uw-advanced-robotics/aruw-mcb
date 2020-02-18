#include "agitator_subsystem.hpp"
#include "src/aruwlib/algorithms/math_user_utils.hpp"
#include <modm/math/filter/pid.hpp>
#include "src/aruwlib/control/subsystem.hpp"
#include "src/aruwlib/motor/dji_motor.hpp"
#include "agitator_rotate_command.hpp"

using namespace aruwlib::motor;

namespace aruwsrc
{

namespace agitator
{
    AgitatorSubsystem::AgitatorSubsystem(
        float kp,
        float ki,
        float kd,
        float maxIAccum,
        float maxOutput,
        float agitatorGearRatio,
        aruwlib::motor::MotorId agitatorMotorId,
        aruwlib::can::CanBus agitatorCanBusId
    ) :
        agitatorPositionPid(kp, ki, kd, maxIAccum, maxOutput),
        agitatorMotor(agitatorMotorId, agitatorCanBusId, false),
        desiredAgitatorAngle(0.0f),
        agitatorCalibratedZeroAngle(0.0f),
        agitatorIsCalibrated(false),
        agitatorJammedTimeout(0),
        agitatorJammedTimeoutPeriod(0),
        gearRatio(agitatorGearRatio)
    {
        agitatorJammedTimeout.stop();
    }

    void AgitatorSubsystem::armAgitatorUnjamTimer(uint32_t predictedRotateTime)
    {
        if (predictedRotateTime == 0)
        {
            agitatorJammedTimeoutPeriod = DEFAULT_AGITATOR_JAMMED_TIMEOUT_PERIOD;
        }
        else
        {
            agitatorJammedTimeoutPeriod = predictedRotateTime;
        }
        agitatorJammedTimeoutPeriod += JAMMED_TOLERANCE_PERIOD;
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
        if (agitatorIsCalibrated)
        {
            agitatorRunPositionPid();
        }
        else
        {
            agitatorCalibrateHere();
        }
    }

    void AgitatorSubsystem::agitatorRunPositionPid()
    {
        if (!agitatorIsCalibrated)
        {
            agitatorPositionPid.reset();
            return;
        }
        agitatorPositionPid.update(desiredAgitatorAngle - getAgitatorAngle());
        agitatorMotor.setDesiredOutput(agitatorPositionPid.getValue());
    }

    bool AgitatorSubsystem::agitatorCalibrateHere()
    {
        if (!agitatorMotor.isMotorOnline())
        {
            return false;
        }
        agitatorCalibratedZeroAngle = getUncalibratedAgitatorAngle();
        agitatorIsCalibrated = true;
        return true;
    }

    float AgitatorSubsystem::getAgitatorAngle() const
    {
        if (!agitatorIsCalibrated)
        {
            return 0.0f;
        }
        return getUncalibratedAgitatorAngle() - agitatorCalibratedZeroAngle;
    }

    float AgitatorSubsystem::getUncalibratedAgitatorAngle() const
    {
        // position is equal to the following equation:
        // position = 2 * PI / encoder resolution * unwrapped encoder value / gear ratio
        return (2.0f * aruwlib::algorithms::PI / static_cast<float>(ENC_RESOLUTION)) *
            agitatorMotor.encStore.getEncoderUnwrapped() / gearRatio;
    }

    void AgitatorSubsystem::setAgitatorDesiredAngle(float newAngle)
    {
        desiredAgitatorAngle = newAngle;
    }

    float AgitatorSubsystem::getAgitatorDesiredAngle() const
    {
        return desiredAgitatorAngle;
    }

    float AgitatorSubsystem::getAgitatorVelocity()
    {
        return agitatorMotor.getShaftRPM() / gearRatio;
    }
}  // namespace agitator

}  // namespace aruwsrc
