#ifndef SUBSYSTEM_MOCK_HPP_
#define SUBSYSTEM_MOCK_HPP_

class SubsystemMock : public aruwlib::control::Subsystem
{
public:
    SubsystemMock(aruwlib::Drivers *drivers) : aruwlib::control::Subsystem(drivers) {}
    MOCK_METHOD(void, initialize, (), (override));
    MOCK_METHOD(void, setDefaultCommand, (Command * defaultCommand), (override));
    MOCK_METHOD(Command *, getDefaultCommand, (), (const override));
    MOCK_METHOD(void, refresh, (), (override));
    MOCK_METHOD(void, onEnterKillMode, (), (override));
    MOCK_METHOD(void, onRefreshKillMode, (), (override));
    MOCK_METHOD(void, onExitKillMode, (), (override));
};  // class SubsystemMock

#endif  // SUBSYSTEM_MOCK_HPP_
