/*----------------------------------------------------------------------------*/
/* Copyright (c) 2011-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "src/control/subsystem.hpp"

namespace aruwlib
{

namespace control
{

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

void Subsystem::InitDefaultCommand() {}

}  // namespace control

}  // namespace aruwlib
