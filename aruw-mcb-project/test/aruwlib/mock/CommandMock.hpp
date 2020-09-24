#ifndef COMMAND_MOCK_HPP_
#define COMMAND_MOCK_HPP_

class CommandMock : public aruwlib::control::Command
{
public:
    CommandMock() = default;
    MOCK_METHOD(const std::set<Subsystem*>&, getRequirements, (), (const override));
    MOCK_METHOD(bool, hasRequirement, (Subsystem * requirement), (const override));
    MOCK_METHOD(void, addSubsystemRequirement, (Subsystem * requirement), (override));
    MOCK_METHOD(const char*, getName, (), (const override));
    MOCK_METHOD(void, initialize, (), (override));
    MOCK_METHOD(void, execute, (), (override));
    MOCK_METHOD(void, end, (bool interrupted), (override));
    MOCK_METHOD(bool, isFinished, (), (const override));
};  // class CommandMock

#endif  // COMMAND_MOCK_HPP_
