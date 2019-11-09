#include "src/control/scheduler.hpp"
#include "src/motor/dji_motor_tx_handler.hpp"
#include "src/communication/can/can_rx_handler.hpp"

namespace aruwlib
{

namespace control
{
    modm::LinkedList<Command*> Scheduler::commandList;

    modm::LinkedList<Subsystem*> Scheduler::subsystemList;

    uint16_t Scheduler::sendReceiveRatio = 1;

    uint16_t Scheduler::updateCounter = 0;

    void Scheduler::motorSendReceiveRatio(uint16_t motorSendReceiveRatio)
    {
        sendReceiveRatio = motorSendReceiveRatio;
    }

    bool Scheduler::addCommand(Command* control)
    {
        const modm::DynamicArray<const Subsystem*>& commandDependencies = control->getRequirements();

        // Check to make sure the control you are trying to add to the scheduler can be added.        
        for (int i = commandDependencies.getSize(); i > 0; i--)
        {
            const Subsystem* subsystemDependency = commandDependencies[i];
            if (
                subsystemDependency->GetCurrentCommand() != nullptr
                && !subsystemDependency->GetCurrentCommand()->isInterruptiable()
            ) {
                return false;
            }
        }

        // // If we can replace the command based of the command dependencies, do so.
        // // O(n^2) :`(
        for (int i = subsystemList.getSize(); i > 0; i--)
        {
            Subsystem* currSubsystem = subsystemList.getFront();
            subsystemList.removeFront();
            subsystemList.append(currSubsystem);
            for (int j = commandDependencies.getSize(); j > 0; j--)
            {
                if (commandDependencies[j] == currSubsystem)
                {
                    currSubsystem->SetCurrentCommand(control);
                }
            }
        }

        commandList.append(control);
        control->initialize();
        return true;
    }

    void Scheduler::run()
    {
        // update dji motors
        aruwlib::can::CanRxHandler::pollCanData();

        // refresh all subsystems (i.e. run control loops where necessary)
        // additionally, check if no command is running, in which case run the
        // default command.
        for (int i = subsystemList.getSize(); i > 0; i--)
        {
            Subsystem* currSubsystem = subsystemList.getFront();
            subsystemList.removeFront();
            subsystemList.append(currSubsystem);
            if (currSubsystem->GetCurrentCommand() == nullptr)
            {
                if (currSubsystem->GetDefaultCommand() != nullptr)
                {
                    currSubsystem->GetDefaultCommand()->schedule();
                }
            }
            currSubsystem->refresh();
        }

        // // loop through commands.
        for (int i = commandList.getSize(); i > 0; i--)
        {
           // modm::SmartPointer currCommand(commandList.getFront());
            Command* currCommand = commandList.getFront();
            commandList.removeFront();
            currCommand->execute();
            // only add back to list if the command is not finished
            if (!currCommand->isFinished())
            {
                commandList.append(currCommand);
            }
        }

        // send stuff to dji motors based on sendReceiveRatio
        if (updateCounter == 0)
        {
            aruwlib::motor::DjiMotorTxHandler::processCanSendData();
        }

        //updateCounter = (updateCounter + 1) % sendReceiveRatio;
    }

    void Scheduler::resetAll(void)
    {
        for (int i = commandList.getSize(); i > 0; i--)
        {
            Command* currCommand = commandList.getFront();
            commandList.removeFront();
            commandList.append(currCommand);
        }
    }

    void Scheduler::removeCommand(const Command* command)
    {
        for (int i = commandList.getSize(); i > 0; i--)
        {
            Command* currCommand = commandList.getFront();
            commandList.removeFront();
            if (currCommand != command)
            {
                commandList.append(currCommand);
            }
        }
    }

    bool Scheduler::isScheduled(const Command* command)
    {
        bool scheduled = false;
        for (int i = commandList.getSize(); i > 0; i--)
        {
            Command* currCommand = commandList.getFront();
            if (command == currCommand)
            {
                scheduled = true;
            }
            commandList.removeFront();
            commandList.append(currCommand);
        }
        return scheduled;
    }

    bool Scheduler::addSubsystem(Subsystem* subsystem)
    {
        bool subsystemAlreadyAdded = false;
        for (int i = subsystemList.getSize(); i > 0; i--)
        {
            Subsystem* currSubsystem = subsystemList.getFront();
            subsystemList.removeFront();
            subsystemList.append(currSubsystem);
            if (currSubsystem == subsystem)
            {
                subsystemAlreadyAdded = true;
            }
        }
        if (!subsystemAlreadyAdded)
        {
            subsystemList.append(subsystem);
        }
        return subsystemAlreadyAdded;
    }
}  // namespace control

}  // namespace aruwlib
