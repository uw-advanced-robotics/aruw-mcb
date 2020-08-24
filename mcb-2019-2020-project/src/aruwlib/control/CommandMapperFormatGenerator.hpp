#ifndef COMMAND_MAPPER_FORMAT_GENERATOR_HPP_
#define COMMAND_MAPPER_FORMAT_GENERATOR_HPP_

#include <string>
#include <string_view>
#include <vector>

#include "aruwlib/communication/remote.hpp"

#include "CommandMapper.hpp"

namespace aruwlib
{
namespace control
{
class CommandMapperFormatGenerator
{
public:
    explicit CommandMapperFormatGenerator(const CommandMapper &mapper) : mapper(mapper) {}
    ~CommandMapperFormatGenerator() = default;

    /**
     * @return A list of mappings in string format, parsed from the CommandMapper
     *      passed into the class.
     * @note This is very slow because of the necessary std::string parsing. Never
     *      call this while performance matters.
     */
    const std::vector<std::string> generateMappings() const;

private:
    const CommandMapper &mapper;

    const std::string formattedMapping(const RemoteMapState &ms) const;
    const std::string formattedMappedCommands(const std::vector<Command *> mc) const;
    constexpr std::string_view switchStateToString(Remote::SwitchState state) const;
    const std::string keyMapToString(uint16_t keys) const;
};  // class CommandMapperFormatGenerator
}  // namespace control
}  // namespace aruwlib

#endif  // COMMAND_MAPPER_FORMAT_GENERATOR_HPP_
