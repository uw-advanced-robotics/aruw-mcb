#ifndef __COMMAND_SENTINEL_DRIVE_RANDOM_HPP__
#define __COMMAND_SENTINEL_DRIVE_RANDOM_HPP__

#include <aruwlib/architecture/timeout.hpp>
#include <aruwlib/control/command.hpp>

#include "sentinel_drive_subsystem.hpp"

namespace aruwsrc
{
namespace control
{
template <typename Drivers>
class SentinelAutoDriveCommand : public aruwlib::control::Command<Drivers>
{
public:
    explicit SentinelAutoDriveCommand(SentinelDriveSubsystem<Drivers>* subsystem)
        : subsystemSentinelDrive(subsystem),
          changeVelocityTimer(CHANGE_TIME_INTERVAL)
    {
        this->addSubsystemRequirement(subsystem);
#ifndef ENV_SIMULATOR
        RandomNumberGenerator::enable();
#endif
    }

    void initialize() override { chosenNewRPM = false; }

    void execute() override
    {
        if (this->changeVelocityTimer.isExpired() || !chosenNewRPM)
        {
#ifdef ENV_SIMULATOR
            chosenNewRPM = true;
#else
            chosenNewRPM = RandomNumberGenerator::isReady();
#endif
            if (chosenNewRPM)
            {
                this->changeVelocityTimer.restart(CHANGE_TIME_INTERVAL);
#ifdef ENV_SIMULATOR
                currentRPM = MIN_RPM + (MAX_RPM - MIN_RPM) / 2;
#else
                uint32_t randVal = RandomNumberGenerator::getValue();
                currentRPM = randVal % (MAX_RPM - MIN_RPM + 1) + MIN_RPM;
                if (randVal % 2 == 0)
                {
                    currentRPM *= -1.0f;
                }
#endif
            }
        }

        // reverse direction if close to the end of the rail
        float curPos = subsystemSentinelDrive->absolutePosition();
        if ((currentRPM < 0 && curPos < RAIL_BUFFER) ||
            (currentRPM > 0 && curPos > SentinelDriveSubsystem<Drivers>::RAIL_LENGTH - RAIL_BUFFER))
        {
            currentRPM = -currentRPM;
        }

        subsystemSentinelDrive->setDesiredRpm(currentRPM);
    }

    void end(bool) override { subsystemSentinelDrive->setDesiredRpm(0); }

    bool isFinished() const override { return false; }

    const char* getName() const override { return "sentinel auto drive command"; }

private:
    static constexpr int16_t MIN_RPM = 5000;
    static constexpr int16_t MAX_RPM = 7000;
    static constexpr int16_t CHANGE_TIME_INTERVAL = 750;
    static constexpr float RAIL_BUFFER = 0.1f * SentinelDriveSubsystem<Drivers>::RAIL_LENGTH;

    float currentRPM = 0;
    bool chosenNewRPM = false;

    SentinelDriveSubsystem<Drivers>* subsystemSentinelDrive;
    aruwlib::arch::MilliTimeout changeVelocityTimer;
};

}  // namespace control

}  // namespace aruwsrc

#endif
