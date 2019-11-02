/**
 * A robot subsystem. Subsystems are the basic unit of robot organization in
 * the Command-based framework; they encapsulate low-level hardware objects
 * (motor controllers, sensors, etc) and provide methods through which they can
 * be used by Commands. Subsystems are used by the CommandScheduler's resource
 * management system to ensure multiple robot actions are not "fighting" over
 * the same hardware; Commands that use a subsystem should include that
 * subsystem in their GetRequirements() method, and resources used within a
 * subsystem should generally remain encapsulated and not be shared by other
 * parts of the robot.
 *
 * <p>Subsystems must be registered with the scheduler with the
 * CommandScheduler.RegisterSubsystem() method in order for the
 * Periodic() method to be called. It is recommended that this method be called
 * from the constructor of users' Subsystem implementations. The
 * SubsystemBase class offers a simple base for user implementations
 * that handles this.
 */

#ifndef __SUBSYSTEM_HPP__
#define __SUBSYSTEM_HPP__

namespace aruwlib
{

namespace control
{

class Command;

class Subsystem {
 public:
    /**
     * Sets the default Command of the subsystem. The default command will be
     * automatically scheduled when no other commands are scheduled that require
     * the subsystem. Default commands should generally not end on their own, i.e.
     * their IsFinished() method should always return false. Will automatically
     * register this subsystem with the CommandScheduler.
     *
     * @param defaultCommand the default command to associate with this subsystem
     */
    void SetDefaultCommand(aruwlib::control::Command* defaultCommand);

    /**
     * Gets the default command for this subsystem. Returns null if no default
     * command is currently associated with the subsystem.
     *
     * @return the default command associated with this subsystem
     */
    Command* GetDefaultCommand() const;

    /**
     * Returns the command currently running on this subsystem.  Returns null if
     * no command is currently scheduled that requires this subsystem.
     *
     * @return the scheduled command currently requiring this subsystem
     */
    Command* GetCurrentCommand() const;

    /**
     * Sets the current command. Only one command can control a subsystem at a time.
     *
     * @param command the new current command
     */
    void SetCurrentCommand(Command* command);

    /**
     * Initialize the default command for this subsystem.
     *
     * This is meant to be the place to call SetDefaultCommand in a subsystem and
     * will be called on all the subsystems by the CommandBase method before the
     * program starts running by using the list of all registered Subsystems
     * inside the Scheduler.
     *
     * This should be overridden by a Subsystem that has a default Command
     */
    virtual void InitDefaultCommand();

    virtual void refresh();
 private:
    Command* defaultCommand = nullptr;

    bool currentCommandChanged = false;

    Command* currentCommand = nullptr;
};

}  // namespace control

}  // namespace aruwlib

#endif
