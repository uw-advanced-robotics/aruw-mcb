#include "engineer_17mm_reservoir_subsystem.hpp"

namespace aruwsrc
{

namespace control
{
    Engineer17mmReservoirSubsystem::Engineer17mmReservoirSubsystem() :
        reservoirMotor(RESERVOIR_MOTOR_ID, CAN_BUS_MOTORS, false),
        reservoirPositionPid(PID_P, PID_I, PID_D, PID_MAX_ERROR_SUM, PID_MAX_OUTPUT),
        desiredReservoirAngle(0.0f),
        reservoirCalibrationAngle(0.0f),
        reservoirIsCalibrated(false)
    {}

    void Engineer17mmReservoirSubsystem::refresh()
    {
        reservoirRunPositionPid();
    }

    void Engineer17mmReservoirSubsystem::setReservoirAngle(float newAngle)
    {
        desiredReservoirAngle = newAngle;
    }

    float Engineer17mmReservoirSubsystem::getReservoirAngle() const
    {
        if (!reservoirIsCalibrated)
        {
            return 0.0f;
        }
        // TODO, angle calculation
    }

    float Engineer17mmReservoirSubsystem::getReservoirDesiredAngle() const
    {
        return desiredReservoirAngle;
    }

    bool Engineer17mmReservoirSubsystem::reservoirCalibrateHere()
    {
        if (!reservoirMotor.isMotorOnline())
        {
            return false;
        }
        // TODO: determine calibration calculation
        reservoirCalibrationAngle = ?;
        reservoirIsCalibrated = true;
        return true;
    }

    void Engineer17mmReservoirSubsystem::reservoirRunPositionPid()
    {
        if (!reservoirIsCalibrated)
        {
            reservoirPositionPid.reset();
            return;
        }
        // TODO: determine calculation
        reservoirPositionPid.update(desiredReservoirAngle - ?);
        reservoirMotor.setDesiredOutput(reservoirPositionPid.getValue());
    }

    float Engineer17mmReservoirSubsystem::getUncalibratedReservoirAngle() const
    {
        // TODO:
    }

}  // namespace control

}  // namespace aruwsrc