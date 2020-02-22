#include "reservoir_42mm_subsystem.hpp"

namespace aruwsrc
{

namespace engineer
{
    Reservoir42mmSubsystem::Reservoir42mmSubsystem() :
        reservoirMotor(RESERVOIR_42MM_MOTOR_ID, CAN_BUS_MOTORS, false),
        positionPid(PID_P, PID_I, PID_D, PID_MAX_ERROR_SUM, PID_MAX_OUTPUT),
        desiredReservoirAngle(0.0f),
        reservoirCalibratedAngle(0.0f),
        reservoirIsCalibrated(false),
        reservoirIsClosed(true)
    {}

    void Reservoir42mmSubsystem::refresh()
    {
        if (reservoirIsCalibrated)
        {
            reservoirRunPositionPid();
        }
        else
        {
            reservoirCalibrateHere();
        }
    }

    float Reservoir42mmSubsystem::getAngle() const
    {
        if (!reservoirIsCalibrated)
        {
            return 0.0f;
        }
        return getUncalibratedReservoirAngle() - reservoirCalibratedAngle;
    }

    float Reservoir42mmSubsystem::getDesiredAngle() const
    {
        return desiredReservoirAngle;
    }

    void Reservoir42mmSubsystem::setDesiredAngle(float newAngle)
    {
        desiredReservoirAngle = newAngle;
    }

    bool Reservoir42mmSubsystem::reservoirCalibrateHere()
    {
        if (!reservoirMotor.isMotorOnline())
        {
            return false;
        }
        reservoirCalibratedAngle = getUncalibratedReservoirAngle();
        reservoirIsCalibrated = true;
        return true;
    }

    void Reservoir42mmSubsystem::reservoirToggleState()
    {
        reservoirIsClosed = !reservoirIsClosed;
    }

    bool Reservoir42mmSubsystem::isClosed()
    {
        return reservoirIsClosed;
    }

    void Reservoir42mmSubsystem::reservoirRunPositionPid()
    {
        if (!reservoirIsCalibrated)
        {
            positionPid.reset();
            return;
        }
        positionPid.update(desiredReservoirAngle - getAngle());
        reservoirMotor.setDesiredOutput(positionPid.getValue());
    }

    float Reservoir42mmSubsystem::getUncalibratedReservoirAngle() const
    {
        return (2.0f * aruwlib::algorithms::PI / static_cast<float>(ENC_RESOLUTION)) *
            reservoirMotor.encStore.getEncoderUnwrapped() / RESERVOIR_42MM_GEAR_RATIO;
    }
}  // namespace engineer

}  // namespace aruwsrc