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
 * Command that is scheduled once that tells the sentry that
 * there is no motion strategy
 */
class NoMotionStrategyCommand : public tap::control::Command
{
public:
    NoMotionStrategyCommand(SentryRequestSubsystem &sentryRequestSubsystem)
        : sub(sentryRequestSubsystem)
    {
        this->addSubsystemRequirement(&sentryRequestSubsystem);
    }

    virtual const char *getName() const { return "sentry no motion strategy"; }
    virtual bool isReady() { return true; }
    virtual void initialize() { this->sub.queueRequest(SentryRequestMessageType::NONE); }
    virtual void execute() {}
    virtual void end(bool) {}
    virtual bool isFinished() const { return true; }

private:
    SentryRequestSubsystem &sub;
};

/**
 * Tells the sentry to go to the friendly base.
 * This command executes once when schedule
 */
class GoToFriendlyBaseCommand : public tap::control::Command
{
public:
    GoToFriendlyBaseCommand(SentryRequestSubsystem &sentryRequestSubsystem)
        : sub(sentryRequestSubsystem)
    {
        this->addSubsystemRequirement(&sentryRequestSubsystem);
    }

    virtual const char *getName() const { return "sentry go to friendly base"; }
    virtual bool isReady() { return true; }
    virtual void initialize()
    {
        this->sub.queueRequest(SentryRequestMessageType::GO_TO_FRIENDLY_BASE);
    }
    virtual void execute() {}
    virtual void end(bool) {}
    virtual bool isFinished() const { return true; }

private:
    SentryRequestSubsystem &sub;
};

/**
 * Tells the sentry to go to the enemy base.
 * This command executes once when schedule
 */
class GoToEnemyBaseCommand : public tap::control::Command
{
public:
    GoToEnemyBaseCommand(SentryRequestSubsystem &sentryRequestSubsystem)
        : sub(sentryRequestSubsystem)
    {
        this->addSubsystemRequirement(&sentryRequestSubsystem);
    }

    virtual const char *getName() const { return "sentry go to enemy base"; }
    virtual bool isReady() { return true; }
    virtual void initialize()
    {
        this->sub.queueRequest(SentryRequestMessageType::GO_TO_ENEMY_BASE);
    }
    virtual void execute() {}
    virtual void end(bool) {}
    virtual bool isFinished() const { return true; }

private:
    SentryRequestSubsystem &sub;
};

/**
 * Tells the sentry to go to the supplier zone
 * This command executes once when schedule
 */
class GoToSupplierZoneCommand : public tap::control::Command
{
public:
    GoToSupplierZoneCommand(SentryRequestSubsystem &sentryRequestSubsystem)
        : sub(sentryRequestSubsystem)
    {
        this->addSubsystemRequirement(&sentryRequestSubsystem);
    }

    virtual const char *getName() const { return "sentry go to supplier zone command"; }
    virtual bool isReady() { return true; }
    virtual void initialize()
    {
        this->sub.queueRequest(SentryRequestMessageType::GO_TO_FRIENDLY_SUPPLIER_ZONE);
    }
    virtual void execute() {}
    virtual void end(bool) {}
    virtual bool isFinished() const { return true; }

private:
    SentryRequestSubsystem &sub;
};

/**
 * Tells the sentry to go to the enemy supplier zone
 * This command executes once when schedule
 */
class GoToEnemySupplierZoneCommand : public tap::control::Command
{
public:
    GoToEnemySupplierZoneCommand(SentryRequestSubsystem &sentryRequestSubsystem)
        : sub(sentryRequestSubsystem)
    {
        this->addSubsystemRequirement(&sentryRequestSubsystem);
    }

    virtual const char *getName() const { return "sentry go to enemy supplier zone command"; }
    virtual bool isReady() { return true; }
    virtual void initialize()
    {
        this->sub.queueRequest(SentryRequestMessageType::GO_TO_ENEMY_SUPPLIER_ZONE);
    }
    virtual void execute() {}
    virtual void end(bool) {}
    virtual bool isFinished() const { return true; }

private:
    SentryRequestSubsystem &sub;
};

/**
 * Tells the sentry to go to the center point
 * This command executes once when schedule
 */
class GoToCenterPointCommand : public tap::control::Command
{
public:
    GoToCenterPointCommand(SentryRequestSubsystem &sentryRequestSubsystem)
        : sub(sentryRequestSubsystem)
    {
        this->addSubsystemRequirement(&sentryRequestSubsystem);
    }

    virtual const char *getName() const { return "sentry go to center point command"; }
    virtual bool isReady() { return true; }
    virtual void initialize()
    {
        this->sub.queueRequest(SentryRequestMessageType::GO_TO_CENTER_POINT);
    }
    virtual void execute() {}
    virtual void end(bool) {}
    virtual bool isFinished() const { return true; }

private:
    SentryRequestSubsystem &sub;
};

/**
 * Tells the sentry to hold fire
 * This command executes once when schedule
 */
class HoldFireCommand : public tap::control::Command
{
public:
    HoldFireCommand(SentryRequestSubsystem &sentryRequestSubsystem) : sub(sentryRequestSubsystem)
    {
        this->addSubsystemRequirement(&sentryRequestSubsystem);
    }

    virtual const char *getName() const { return "sentry hold fire command"; }
    virtual bool isReady() { return true; }
    virtual void initialize() { this->sub.queueRequest(SentryRequestMessageType::HOLD_FIRE); }
    virtual void execute() {}
    virtual void end(bool) {}
    virtual bool isFinished() const { return true; }

private:
    SentryRequestSubsystem &sub;
};

/**
 * Tells the sentry to toggle moving
 * This command executes once when schedule
 */
class ToggleMovementCommand : public tap::control::Command
{
public:
    ToggleMovementCommand(SentryRequestSubsystem &sentryRequestSubsystem)
        : sub(sentryRequestSubsystem)
    {
        this->addSubsystemRequirement(&sentryRequestSubsystem);
    }

    virtual const char *getName() const { return "sentry toggle moving command"; }
    virtual bool isReady() { return true; }
    virtual void initialize() { this->sub.queueRequest(SentryRequestMessageType::TOGGLE_MOVEMENT); }
    virtual void execute() {}
    virtual void end(bool) {}
    virtual bool isFinished() const { return true; }

private:
    SentryRequestSubsystem &sub;
};

/**
 * Tells the sentry to toggle beyblading
 * This command executes once when schedule
 */
class ToggleBeybladeCommand : public tap::control::Command
{
public:
    ToggleBeybladeCommand(SentryRequestSubsystem &sentryRequestSubsystem)
        : sub(sentryRequestSubsystem)
    {
        this->addSubsystemRequirement(&sentryRequestSubsystem);
    }

    virtual const char *getName() const { return "sentry toggle beyblading command"; }
    virtual bool isReady() { return true; }
    virtual void initialize() { this->sub.queueRequest(SentryRequestMessageType::TOGGLE_BEYBLADE); }
    virtual void execute() {}
    virtual void end(bool) {}
    virtual bool isFinished() const { return true; }

private:
    SentryRequestSubsystem &sub;
};
}  // namespace aruwsrc::communication::serial

#endif  //  SENTRY_REQUEST_COMMANDS_HPP_
