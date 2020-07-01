#include "CommandMapperFormatGenerator.hpp"

namespace aruwlib
{
namespace control
{
std::vector<std::string> CommandMapperFormatGenerator::generateMappings() const
{
    std::vector<std::string> out;

    for (const auto &mapping : mapper.commandsToRun)
    {
        out.push_back(
            formattedMapping(mapping->mapState) + ":\t" +
            formattedMappedCommands(mapping->mappedCommands));
    }
    return out;
}

std::string CommandMapperFormatGenerator::formattedMapping(const RemoteMapState &ms) const
{
    std::string out = "[";
    if (ms.useKeys)
    {
        out += "keys: ";
        out += keyMapToString(ms.keys);
        out += ", ";
    }
    if (ms.useNegKeys)
    {
        out += "neg keys: ";
        out += keyMapToString(ms.negKeys);
        out += ", ";
    }
    if (ms.useLMouseButton)
    {
        out += "left mouse pressed, ";
    }
    if (ms.useRMouseButton)
    {
        out += "right mouse pressed, ";
    }
    if (ms.useLSwitch)
    {
        out += "left switch: ";
        out += switchStateToString(ms.lSwitch);
        out += ", ";
    }
    if (ms.useRSwitch)
    {
        out += "right switch: ";
        out += switchStateToString(ms.rSwitch);
        out += ", ";
    }
    out = out.length() == 1 ? " none " : out.substr(0, out.length() - 2);
    return out + "]";
}

std::string CommandMapperFormatGenerator::formattedMappedCommands(
    const std::vector<Command *> mc) const
{
    std::string out;
    out += "[";
    bool first = true;
    for (const Command *cmd : mc)
    {
        if (!first)
        {
            out += ", ";
        }
        out += cmd->getName();
        first = false;
    }
    return out + "]";
}

std::string CommandMapperFormatGenerator::switchStateToString(Remote::SwitchState state) const
{
    switch (state)
    {
        case Remote::SwitchState::DOWN:
            return "down";
        case Remote::SwitchState::MID:
            return "mid";
        case Remote::SwitchState::UNKNOWN:
            return "unknown";
        case Remote::SwitchState::UP:
            return "up";
        default:
            return "";
    }
}

std::string CommandMapperFormatGenerator::keyMapToString(uint16_t keys) const
{
    if (keys == 0)
    {
        return "{ NONE }";
    }
    std::string out;
    out = "{";
    if (keys & (1 << static_cast<int>(Remote::Key::A)))
    {
        out += "A, ";
    }
    if (keys & (1 << static_cast<int>(Remote::Key::B)))
    {
        out += "B, ";
    }
    if (keys & (1 << static_cast<int>(Remote::Key::C)))
    {
        out += "C, ";
    }
    if (keys & (1 << static_cast<int>(Remote::Key::D)))
    {
        out += "D, ";
    }
    if (keys & (1 << static_cast<int>(Remote::Key::E)))
    {
        out += "E, ";
    }
    if (keys & (1 << static_cast<int>(Remote::Key::F)))
    {
        out += "F, ";
    }
    if (keys & (1 << static_cast<int>(Remote::Key::G)))
    {
        out += "G, ";
    }
    if (keys & (1 << static_cast<int>(Remote::Key::Q)))
    {
        out += "Q, ";
    }
    if (keys & (1 << static_cast<int>(Remote::Key::R)))
    {
        out += "R, ";
    }
    if (keys & (1 << static_cast<int>(Remote::Key::S)))
    {
        out += "S, ";
    }
    if (keys & (1 << static_cast<int>(Remote::Key::V)))
    {
        out += "V, ";
    }
    if (keys & (1 << static_cast<int>(Remote::Key::W)))
    {
        out += "W, ";
    }
    if (keys & (1 << static_cast<int>(Remote::Key::X)))
    {
        out += "X, ";
    }
    if (keys & (1 << static_cast<int>(Remote::Key::Z)))
    {
        out += "Z, ";
    }
    if (keys & (1 << static_cast<int>(Remote::Key::SHIFT)))
    {
        out += "SHIFT, ";
    }
    if (keys & (1 << static_cast<int>(Remote::Key::CTRL)))
    {
        out += "CTRL, ";
    }
    return out.substr(0, out.length() - 2) + "}";
}
}  // namespace control
}  // namespace aruwlib
