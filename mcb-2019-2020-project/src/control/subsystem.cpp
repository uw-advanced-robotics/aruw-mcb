/*----------------------------------------------------------------------------*/
/* Copyright (c) 2011-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "src/control/subsystem.hpp"
#include "src/control/command.hpp"

namespace aruwlib
{

namespace control
{
    Subsystem::~Subsystem()
    {
        defaultCommand->~Command();
        currentCommand->~Command();
    }

    void Subsystem::SetDefaultCommand(Command* command)
    {
        defaultCommand = command;
    }

    Command* Subsystem::GetDefaultCommand() const
    {
        return defaultCommand;
    }

    void Subsystem::SetCurrentCommand(Command* command)
    {
        currentCommand = command;
    }

    Command* Subsystem::GetCurrentCommand() const
    {
        return currentCommand;
    }

    void Subsystem::removeCurrentCommand()
    {
        currentCommand = nullptr;
    }
}  // namespace control

}  // namespace aruwlib
