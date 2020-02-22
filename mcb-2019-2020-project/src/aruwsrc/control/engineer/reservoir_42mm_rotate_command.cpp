#include "reservoir_42mm_rotate_command.hpp"

namespace aruwsrc
{

namespace engineer
{
    Reservoir42mmRotateCommand::Reservoir42mmRotateCommand(
        Reservoir42mmSubsystem* reservoir,
        float reservoirAngleChange,
        float reservoirRotateTime) :
        reservoirTargetChange(reservoirAngleChange),
        reservoirRotateSetpoint(
            RESERVOIR_ROTATE_COMMAND_PERIOD * reservoirAngleChange / reservoirRotateTime,
            RESERVOIR_ROTATE_COMMAND_PERIOD * reservoirAngleChange / reservoirRotateTime, 0),
        reservoirDesiredRotateTime(reservoirRotateTime),
        reservoirMinRotateTime(RESERVOIR_MIN_ROTATE_TIME)
    {
        this->addSubsystemRequirement(dynamic_cast<aruwlib::control::Subsystem*>(reservoir));
        connectedReservoir = reservoir;
    }

    void Reservoir42mmRotateCommand::initialize()
    {
        reservoirRotateSetpoint.reset(connectedReservoir->getAngle());
        reservoirRotateSetpoint.setTarget(connectedReservoir->getDesiredAngle() + reservoirTargetChange);

        reservoirMinRotateTime.restart(RESERVOIR_MIN_ROTATE_TIME);
    }

    void Reservoir42mmRotateCommand::execute()
    {
        reservoirRotateSetpoint.update();

        connectedReservoir->setDesiredAngle(reservoirRotateSetpoint.getValue());
    }

    void Reservoir42mmRotateCommand::end(bool interrupted)
    {
        connectedReservoir->setDesiredAngle(connectedReservoir->getDesiredAngle());
    }

    bool Reservoir42mmRotateCommand::isFinished() const
    {
        return fabs(static_cast<double>(connectedReservoir->getAngle()
         - connectedReservoir->getDesiredAngle()))
         < static_cast<double>(RESERVOIR_SETPOINT_TOLERANCE)
         && reservoirRotateSetpoint.isTargetReached()
         && reservoirMinRotateTime.isExpired();
    }
}  // namespace engineer

}  // namespace aruwsrc