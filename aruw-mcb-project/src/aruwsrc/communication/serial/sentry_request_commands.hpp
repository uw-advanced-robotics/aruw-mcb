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

#ifndef SENTRY_REQUEST_COMMANDS_HPP_
#define SENTRY_REQUEST_COMMANDS_HPP_

#include "tap/control/command.hpp"

#include "sentry_request_subsystem.hpp"

namespace aruwsrc::communication::serial
{
/**
 * Command that is scheduled once that queues the pause projectile launching
 * SentryRequestMessageType.
 * @todo: new commands, deprecate these
 */
class PauseProjectileLaunchingCommand : public tap::control::Command
{
public:
    PauseProjectileLaunchingCommand(SentryRequestSubsystem &sentryRequestSubsystem)
        : sub(sentryRequestSubsystem)
    {
        this->addSubsystemRequirement(&sentryRequestSubsystem);
    }

    virtual const char *getName() const { return "pause projectile launching"; }
    virtual bool isReady() { return true; }
    virtual void initialize()
    {
        this->sub.queueRequest(SentryRequestMessageType::PAUSE_PROJECTILE_LAUNCHING);
    }
    virtual void execute() {}
    virtual void end(bool) {}
    virtual bool isFinished() const { return true; }

private:
    SentryRequestSubsystem &sub;
};

/**
 * Command that is scheduled once that queues the toggle drive movement SentryRequestMessageType.
 */
class ToggleDriveMovementCommand : public tap::control::Command
{
public:
    ToggleDriveMovementCommand(SentryRequestSubsystem &sentryRequestSubsystem)
        : sub(sentryRequestSubsystem)
    {
        this->addSubsystemRequirement(&sentryRequestSubsystem);
    }

    virtual const char *getName() const { return "toggle drive movement"; }
    virtual bool isReady() { return true; }
    virtual void initialize()
    {
        this->sub.queueRequest(SentryRequestMessageType::TOGGLE_DRIVE_MOVEMENT);
    }
    virtual void execute() {}
    virtual void end(bool) {}
    virtual bool isFinished() const { return true; }

private:
    SentryRequestSubsystem &sub;
};

/**
 * Command that is scheduled once and queues the select new robot SentryRequestMessageType.
 */
class SelectNewRobotCommand : public tap::control::Command
{
public:
    SelectNewRobotCommand(SentryRequestSubsystem &sentryRequestSubsystem)
        : sub(sentryRequestSubsystem)
    {
        addSubsystemRequirement(&sentryRequestSubsystem);
    }

    virtual const char *getName() const { return "select new robot"; }
    virtual bool isReady() { return true; }
    virtual void initialize() { sub.queueRequest(SentryRequestMessageType::SELECT_NEW_ROBOT); }
    virtual void execute() {}
    virtual void end(bool) {}
    virtual bool isFinished() const { return true; }

private:
    SentryRequestSubsystem &sub;
};

/**
 * Command that is scheduled once that queues the target new quadrant SentryRequestMessageType.
 */
class TargetNewQuadrantCommand : public tap::control::Command
{
public:
    TargetNewQuadrantCommand(SentryRequestSubsystem &sentryRequestSubsystem)
        : sub(sentryRequestSubsystem)
    {
        addSubsystemRequirement(&sentryRequestSubsystem);
    }

    virtual const char *getName() const { return "target new quadrant"; }
    virtual bool isReady() { return true; }
    virtual void initialize() { sub.queueRequest(SentryRequestMessageType::TARGET_NEW_QUADRANT); }
    virtual void execute() {}
    virtual void end(bool) {}
    virtual bool isFinished() const { return true; }

private:
    SentryRequestSubsystem &sub;
};
}  // namespace aruwsrc::communication::serial

#endif  //  SENTRY_REQUEST_COMMANDS_HPP_
