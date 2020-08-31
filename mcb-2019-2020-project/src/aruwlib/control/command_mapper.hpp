#ifndef COMMAND_MAPPER_HPP_
#define COMMAND_MAPPER_HPP_

#include <list>
#include <map>
#include <numeric>
#include <utility>

#include "aruwlib/communication/remote.hpp"
#include "aruwlib/errors/create_errors.hpp"

#include "command.hpp"
#include "command_scheduler.hpp"

namespace aruwlib
{
namespace control
{
/**
 * A struct that represents a particular remote state. This consists of
 * a left and rgith swtich state, as well as some particular key state.
 * A RemoteMap is mapped to a particular Command. When the RemoteMap
 * and Command pair is placed in the CommandMapper, the CommandMapper
 * will initiate the Command when the RemoteMap matches the remote's
 * state.
 */
struct RemoteMap
{
    const remote::SwitchState lSwitch;
    const remote::SwitchState rSwitch;
    const uint16_t keys;

    RemoteMap(remote::SwitchState ls, remote::SwitchState rs, uint16_t k)
        : lSwitch(ls),
          rSwitch(rs),
          keys(k)
    {
    }
};

/**
 * Control for mapping commands to actions. For example, all the remote
 * mappings will be handled here. One creates a new RemoteMap
 * using the RemoteMapper class (i.e. press a button on the
 * keyboard) and a pointer to a Command to be executed when commanded.
 *
 * For example, given the command `coolCommand`, to map a hold mapping
 * to the left switch in the up position, we call
 * `addHoldMapping(newKeyMap(aruwilb::remote::Switch::LEFT_SWITCH,
 * aruwilb::remote::SwitchState::UP), &coolCommand);`
 *
 * Currently, a single remote mapping can map to a single Command (i.e.
 * the same switch can't control the executino of multiple Commands).
 *
 * Also, key negations have not yet been implemented (there is no way
 * to for example have a Command stop executing if a certain key is pressed).
 */
template <typename Drivers>
class CommandMapper
{
public:
    CommandMapper() = default;
    CommandMapper(CommandMapper&) = delete;
    CommandMapper& operator=(CommandMapper&) = delete;

    /**
     * Attaches a Command to a remote control mapping which is added to the
     * CommandScheduler once each time the mapping is satisfied.
     *
     * @param[in] mapping a particular remote mapping associated with the Command.
     * @param[in] Command the Command to be triggered by this mapping.
     */
    void addPressMapping(RemoteMap* mapping, Command<Drivers>* command)
    {
        addMap(mapping, new MapInfo(PRESS, command));
    }

    /**
     * Attaches a Command to a remote control mapping which is added to the
     * CommandScheduler once when the mapping is satisfied and is removed
     * when the mapping is stopped being satisfied.
     *
     * @param[in] mapping a particular remote mapping associated with the Command.
     * @param[in] Command the Command to be triggered by this mapping.
     */
    void addHoldMapping(RemoteMap* mapping, Command<Drivers>* command)
    {
        addMap(mapping, new CommandMapper::MapInfo(HOLD, command));
    }

    /**
     * Attaches a Command to a remote control mapping which is added to
     * the CommandScheduler when the remote state matches the passed in
     * mapping, starting the Command over while the mapping is still met
     * if it ever finishes.
     *
     * @param[in] mapping a particular remote mapping associated with the Command.
     * @param[in] Command the Command to be triggered by this mapping.
     */
    void addHoldRepeatMapping(RemoteMap* mapping, Command<Drivers>* command)
    {
        addMap(mapping, new CommandMapper::MapInfo(HOLD_REPEAT, command));
    }

    /**
     * Attaches a Command to a remote control mapping which adds the command
     * to the CommandScheduler whenever a mapping is toggled.
     *
     * @param[in] mapping a particular remote mapping associated with the Command.
     * @param[in] Command the Command to be triggered by this mapping.
     * @note a toggle mapping is interrupted when the key is untoggled.
     */
    void addToggleMapping(RemoteMap* mapping, Command<Drivers>* command)
    {
        addMap(mapping, new CommandMapper::MapInfo(TOGGLE, command));
    }

    /**
     * @return a RemoteMap with dependencies on specified keys and no switches.
     */
    static RemoteMap* newKeyMap(std::list<aruwlib::remote::Key> keySet)
    {
        return newKeyMap(
            aruwlib::remote::SwitchState::UNKNOWN,
            aruwlib::remote::SwitchState::UNKNOWN,
            keySet);
    }

    /**
     * @return a RemoteMap with dependencies on one specified switch and given keys.
     */
    static RemoteMap* newKeyMap(
        aruwlib::remote::Switch sw,
        aruwlib::remote::SwitchState switchState,
        std::list<aruwlib::remote::Key> keySet = {})
    {
        if (sw == aruwlib::remote::Switch::LEFT_SWITCH)
        {
            return newKeyMap(switchState, aruwlib::remote::SwitchState::UNKNOWN, keySet);
        }
        else if (sw == aruwlib::remote::Switch::RIGHT_SWITCH)
        {
            return newKeyMap(aruwlib::remote::SwitchState::UNKNOWN, switchState, keySet);
        }

        RAISE_ERROR(
            "adding a key map with unknown switch state",
            aruwlib::errors::CONTROLLER_MAPPER,
            aruwlib::errors::INVALID_KEY_MAP_TYPE)

        return nullptr;
    }

    /**
     * @return a RemoteMap with dependencies on both specified switches and given keys.
     */
    static RemoteMap* newKeyMap(
        aruwlib::remote::SwitchState leftSwitchState,
        aruwlib::remote::SwitchState rightSwitchState,
        std::list<aruwlib::remote::Key> keySet = {})
    {
        uint16_t keys =
            std::accumulate(keySet.begin(), keySet.end(), 0, [](int acc, aruwlib::remote::Key key) {
                return acc |= 1 << static_cast<uint16_t>(key);
            });

        RemoteMap* ret = new RemoteMap(leftSwitchState, rightSwitchState, keys);
        return ret;
    }

    /**
     * Iterates through all the current mappings to see which buttons are pressed
     * in order to determine which commands should be added to the scheduler. Then,
     * for each button pressed/combination of buttons, executes the commands.
     */
    void handleKeyStateChange(
        uint16_t key,
        aruwlib::remote::SwitchState leftSwitch,
        aruwlib::remote::SwitchState rightSwitch)
    {
        for (std::pair<RemoteMap*, MapInfo*> it : remoteMappings)
        {
            RemoteMap* rm = it.first;
            CommandMapper::MapInfo* mi = it.second;

            bool triggeredCommandStateChange =
                ((leftSwitch == rm->lSwitch ||
                  rm->lSwitch == aruwlib::remote::SwitchState::UNKNOWN) &&
                 (rightSwitch == rm->rSwitch ||
                  rm->rSwitch == aruwlib::remote::SwitchState::UNKNOWN)) &&
                ((key & rm->keys) == rm->keys);

            // adding command when remote things are switched to matching position
            // remote switches and key presses are independent. running a command depends upon one
            // or another or both
            if (triggeredCommandStateChange)
            {
                switch (mi->type)
                {
                    case PRESS:
                        if (!mi->pressed)
                        {
                            mi->pressed = true;
                            Drivers::commandScheduler.addCommand(mi->command);
                        }
                        break;
                    case HOLD:
                        if (!mi->pressed)
                        {
                            Drivers::commandScheduler.addCommand(mi->command);
                            mi->pressed = true;
                        }
                        break;
                    case HOLD_REPEAT:  // spam add the command
                        if (!Drivers::commandScheduler.isCommandScheduled(mi->command))
                        {
                            Drivers::commandScheduler.addCommand(mi->command);
                        }
                        break;
                    case TOGGLE:
                        if (!mi->pressed)
                        {
                            if (mi->toggled)
                            {
                                Drivers::commandScheduler.removeCommand(mi->command, true);
                                mi->toggled = false;
                            }
                            else
                            {
                                Drivers::commandScheduler.addCommand(mi->command);
                                mi->toggled = true;
                            }
                            mi->pressed = true;
                        }
                        break;
                    default:
                        break;
                }
            }
            else
            {
                if ((mi->type == HOLD && mi->pressed) || (mi->type == HOLD_REPEAT))
                {
                    Drivers::commandScheduler.removeCommand(mi->command, true);
                }
                mi->pressed = false;
            }
        }
    }

private:
    /**
     * - PRESS: The Command is added exactly once when you enter the state that matches
     *      the correct state.
     * - HOLD: The Command is added once when the state matches the correct and removed
     *      when you leave this state.
     * - HOLD_REPEAT: The Command is added when the state matches the correct state and
     *      is added again every time the Command ends.
     * - TOGGLE: The Command is added when the state matches the correct state and
     *      removed when you reenter the state again.
     */
    enum MapType
    {
        HOLD_REPEAT,
        HOLD,
        PRESS,
        TOGGLE
    };

    struct MapInfo
    {
        bool pressed = false;
        bool toggled = false;
        MapType type;
        Command<Drivers>* command;
        MapInfo(MapType mt, Command<Drivers>* sp) : type(mt), command(sp) {}
    };

    struct compareRemoteMapPtrs
    {
        bool operator()(const RemoteMap* a, const RemoteMap* b) const
        {
            return a->keys != b->keys || a->lSwitch != b->lSwitch || a->rSwitch != b->rSwitch;
        }
    };

    std::map<RemoteMap*, MapInfo*, compareRemoteMapPtrs> remoteMappings;

    void addMap(RemoteMap* mapping, MapInfo* mapInfo)
    {
        if (remoteMappings.insert(std::pair<RemoteMap*, MapInfo*>(mapping, mapInfo)).second ==
            false)
        {
            RAISE_ERROR(
                "failed to insert io mapping",
                aruwlib::errors::CONTROLLER_MAPPER,
                aruwlib::errors::INVALID_ADD)
            // throw exception here?
        }
    }
};  // class CommandMapper

}  // namespace control

}  // namespace aruwlib

#endif  // COMMAND_MAPPER_HPP_
