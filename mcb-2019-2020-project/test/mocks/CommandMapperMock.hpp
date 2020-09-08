#ifndef COMMAND_MAPPER_MOCK_HPP_
#define COMMAND_MAPPER_MOCK_HPP_

#include <aruwlib/control/command.hpp>
#include <aruwlib/control/command_mapper.hpp>
#include <gmock/gmock.h>

class CommandMapperMock : public aruwlib::control::CommandMapper
{
public:
    MOCK_METHOD(
        void,
        addPressMapping,
        (aruwlib::control::CommandMapper::RemoteMap * mapping, aruwlib::control::Command* Command),
        (override));
    MOCK_METHOD(
        void,
        addHoldMapping,
        (aruwlib::control::CommandMapper::RemoteMap * mapping, aruwlib::control::Command* Command),
        (override));
    MOCK_METHOD(
        void,
        addHoldRepeatMapping,
        (aruwlib::control::CommandMapper::RemoteMap * mapping, aruwlib::control::Command* Command),
        (override));
    MOCK_METHOD(
        void,
        addToggleMapping,
        (aruwlib::control::CommandMapper::RemoteMap * mapping, aruwlib::control::Command* Command),
        (override));
};  // class CommandMapperMock

#endif  // COMMAND_MAPPER_MOCK_HPP_
