#include "engineer_17mm_reservoir_rotate_command.hpp"

namespace aruwsrc
{

namespace control
{
    Engineer17mmReservoirRotateCommand::Engineer17mmReservoirRotateCommand(
        Engineer17mmReservoirSubsystem* reservoir,
            float reservoirAngleChange,
            float reservoirRotateTime) :
            reservoirTargetChange(reservoirAngleChange),
            reservoirRotateSetpoint(
                RESERVOIR_ROTATE_COMMAND_PERIOD * reservoirAngleChange / reservoirRotateTime,
                RESERVOIR_ROTATE_COMMAND_PERIOD * reservoirAngleChange / reservoirRotateTime, 0),
            reservoirDesiredRotateTime(reservoirRotateTime),
            reservoirMinRotateTime(RESERVOIR_MIN_ROTATE_TIME)
        {
            this->addSubsystemRequirement(reinterpret_cast<aruwlib::control::Subsystem*>(reservoir));
            connectedReservoir = reservoir;
        }

    void Engineer17mmReservoirRotateCommand::initialize()
    {
        reservoirRotateSetpoint.reset(connectedReservoir->getReservoirAngle());
        reservoirRotateSetpoint.setTarget(connectedReservoir->getReservoirAngle() + reservoirTargetChange);

        reservoirMinRotateTime.restart(RESERVOIR_MIN_ROTATE_TIME);
    }

    void Engineer17mmReservoirRotateCommand::execute()
    {
        reservoirRotateSetpoint.update();
        connectedReservoir->setReservoirAngle(reservoirRotateSetpoint.getValue());
    }

    void Engineer17mmReservoirRotateCommand::end(bool interrupted)
    {
        // nothing yet??
    }

    bool Engineer17mmReservoirRotateCommand::isFinished() const
    {
        return fabs(static_cast<double>(connectedReservoir->getReservoirAngle()
         - connectedReservoir->getReservoirDesiredAngle()))
         < static_cast<double>(RESERVOIR_SETPOINT_TOLERANCE)
         && reservoirRotateSetpoint.isTargetReached()
         && reservoirMinRotateTime.isExpired();
    }
}  // namespace control

}  // namespace aruwsrc