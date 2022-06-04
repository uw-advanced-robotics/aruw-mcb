/*
 * Copyright (c) 2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruw-mcb.
 *
 * aruw-mcb is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aruw-mcb is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aruw-mcb.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef SENTINEL_REQUEST_COMMANDS_HPP_
#define SENTINEL_REQUEST_COMMANDS_HPP_

#include "tap/control/command.hpp"

#include "sentinel_request_subsystem.hpp"

namespace aruwsrc::communication::serial
{
class ToggleDriveMovementCommand : public tap::control::Command
{
public:
    ToggleDriveMovementCommand(SentinelRequestSubsystem *sentinelRequestSubsystem)
        : sub(sentinelRequestSubsystem)
    {
        this->addSubsystemRequirement(sentinelRequestSubsystem);
    }

    virtual const char *getName() const { return "toggle drive movement"; }
    virtual bool isReady() { return true; }
    virtual void initialize()
    {
        sub->queueRequest(SentinelRequestMessageType::TOGGLE_DRIVE_MOVEMENT);
    }
    virtual void execute() {}
    virtual void end(bool) {}
    virtual bool isFinished() const { return true; }

private:
    SentinelRequestSubsystem *sub;
};

class SelectNewRobotCommand : public tap::control::Command
{
public:
    SelectNewRobotCommand(SentinelRequestSubsystem *sentinelRequestSubsystem)
        : sub(sentinelRequestSubsystem)
    {
        addSubsystemRequirement(sentinelRequestSubsystem);
    }

    virtual const char *getName() const { return "select new robot"; }
    virtual bool isReady() { return true; }
    virtual void initialize() { sub->queueRequest(SentinelRequestMessageType::SELECT_NEW_ROBOT); }
    virtual void execute() {}
    virtual void end(bool) {}
    virtual bool isFinished() const { return true; }

private:
    SentinelRequestSubsystem *sub;
};

class TargetNewQuadrantCommand : public tap::control::Command
{
public:
    TargetNewQuadrantCommand(SentinelRequestSubsystem *sentinelRequestSubsystem)
        : sub(sentinelRequestSubsystem)
    {
        addSubsystemRequirement(sentinelRequestSubsystem);
    }

    virtual const char *getName() const { return "target new quadrant"; }
    virtual bool isReady() { return true; }
    virtual void initialize()
    {
        sub->queueRequest(SentinelRequestMessageType::TARGET_NEW_QUADRANT);
    }
    virtual void execute() {}
    virtual void end(bool) {}
    virtual bool isFinished() const { return true; }

private:
    SentinelRequestSubsystem *sub;
};
}  // namespace aruwsrc::communication::serial

#endif  //  SENTINEL_REQUEST_COMMANDS_HPP_
