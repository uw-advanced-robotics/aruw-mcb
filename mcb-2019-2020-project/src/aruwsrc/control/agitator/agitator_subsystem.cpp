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
    AgitatorSubsystem::AgitatorSubsystem(AgitatorType type) :
        agitatorType(type),
        agitatorMotor(AGITATOR_MOTOR_ID, AGITATOR_MOTOR_CAN_BUS, false),
        desiredAgitatorAngle(0.0f),
        agitatorCalibratedZeroAngle(0.0f),
        agitatorIsCalibrated(false),
        agitatorPositionPid(0.0f, 0.0f, 0.0f, 0.0f, PID_MAX_OUT)
    {
        modm::Pid<float>::Parameter* param = nullptr;   // assign nullptr to fix build
        switch (type)
        {
            case AgitatorType::Soldier:
                param = new modm::Pid<float>::Parameter(PID_P, PID_I, PID_D, PID_MAX_ERR_SUM,
                                                            PID_MAX_OUT);
                break;
            case AgitatorType::Hero1:
                param = new modm::Pid<float>::Parameter(PID_HERO1_P, PID_HERO1_I, PID_HERO1_D,
                                                            PID_HERO1_MAX_ERR_SUM, PID_MAX_OUT);
                break;
            case AgitatorType::Hero2:
                param = new modm::Pid<float>::Parameter(PID_HERO2_P, PID_HERO2_I, PID_HERO2_D,
                                                            PID_HERO2_MAX_ERR_SUM, PID_MAX_OUT);
                break;
        }
        agitatorPositionPid.setParameter(*param);
        delete param;   /// \todo make sure this works
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

    uint32_t t1;

    bool AgitatorSubsystem::isAgitatorJammed()
    {
        t1 = agitatorJammedTimeout.remaining();
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
            agitatorMotor.encStore.getEncoderUnwrapped() / AGITATOR_GEAR_RATIO;
    }

    void AgitatorSubsystem::setAgitatorAngle(float newAngle)
    {
        desiredAgitatorAngle = newAngle;
    }

    float AgitatorSubsystem::getAgitatorDesiredAngle() const
    {
        return desiredAgitatorAngle;
    }
}  // namespace control

}  // namespace aruwsrc
