#include "RemoteMapState.hpp"

#include <numeric>

#include "aruwlib/errors/create_errors.hpp"

namespace aruwlib
{
namespace control
{
RemoteMapState::RemoteMapState(Remote::Switch swh, Remote::SwitchState switchState)
{
    if (swh == Remote::Switch::LEFT_SWITCH)
    {
        initLSwitch(switchState);
    }
    else
    {
        initRSwitch(switchState);
    }
}

RemoteMapState::RemoteMapState(Remote::SwitchState leftss, Remote::SwitchState rightss)
{
    initLSwitch(leftss);
    initRSwitch(rightss);
}

RemoteMapState::RemoteMapState(
    const std::list<Remote::Key> &keySet,
    const std::list<Remote::Key> &negKeySet)
{
    initKeys(keySet);
    initNegKeys(negKeySet);
}

RemoteMapState::RemoteMapState(MouseButton button)
{
    if (button == MouseButton::LEFT)
    {
        initLMouseButton();
    }
    else
    {
        initRMouseButton();
    }
}

void RemoteMapState::initLSwitch(Remote::SwitchState ss)
{
    if (ss == Remote::SwitchState::UNKNOWN)
    {
        return;
    }
    useLSwitch = true;
    lSwitch = ss;
}

void RemoteMapState::initRSwitch(Remote::SwitchState ss)
{
    if (ss == Remote::SwitchState::UNKNOWN)
    {
        return;
    }
    useRSwitch = true;
    rSwitch = ss;
}

void RemoteMapState::initKeys(uint16_t keys)
{
    if (keys == 0)
    {
        return;
    }
    if (useNegKeys && ((this->negKeys & keys) != 0))
    {
        return;
    }
    useKeys = true;
    this->keys = keys;
}

void RemoteMapState::initNegKeys(uint16_t negKeys)
{
    if (negKeys == 0)
    {
        return;
    }
    if (useKeys && ((this->keys & negKeys) != 0))
    {
        return;
    }
    useNegKeys = true;
    this->negKeys = negKeys;
}

void RemoteMapState::initKeys(const std::list<Remote::Key> &keySet)
{
    uint16_t keys = std::accumulate(keySet.begin(), keySet.end(), 0, [](int acc, Remote::Key key) {
        return acc |= 1 << static_cast<uint16_t>(key);
    });

    initKeys(keys);
}

void RemoteMapState::initNegKeys(const std::list<Remote::Key> &negKeySet)
{
    // extract a bit form of the key set.
    uint16_t negKeys =
        std::accumulate(negKeySet.begin(), negKeySet.end(), 0, [](int acc, Remote::Key key) {
            return acc |= 1 << static_cast<uint16_t>(key);
        });
    initNegKeys(negKeys);
}

void RemoteMapState::initLMouseButton()
{
    useLMouseButton = true;
    lMouseButton = true;
}

void RemoteMapState::initRMouseButton()
{
    useRMouseButton = true;
    rMouseButton = true;
}

bool RemoteMapState::stateSubset(const RemoteMapState &other) const
{
    if (useRSwitch && rSwitch != other.rSwitch)
    {
        return false;
    }
    if (useLSwitch && lSwitch != other.lSwitch)
    {
        return false;
    }
    if (useKeys && ((keys & other.keys) != keys))
    {
        return false;
    }
    if (useLMouseButton && lMouseButton != other.lMouseButton)
    {
        return false;
    }
    if (useRMouseButton && rMouseButton != other.rMouseButton)
    {
        return false;
    }
    return true;
}

bool operator==(const RemoteMapState &rms1, const RemoteMapState &rms2)
{
    return rms1.useLSwitch == rms2.useLSwitch && rms1.lSwitch == rms2.lSwitch &&
           rms1.useRSwitch == rms2.useRSwitch && rms1.rSwitch == rms2.rSwitch &&
           rms1.useKeys == rms2.useKeys && rms1.keys == rms2.keys &&
           rms1.useNegKeys == rms2.useNegKeys && rms1.negKeys == rms2.negKeys &&
           rms1.useLMouseButton == rms2.useLMouseButton && rms1.lMouseButton == rms2.lMouseButton &&
           rms1.useRMouseButton == rms2.useRMouseButton && rms1.rMouseButton == rms2.rMouseButton;
}

bool operator!=(const RemoteMapState &rms1, const RemoteMapState &rms2) { return !(rms1 == rms2); }

}  // namespace control
}  // namespace aruwlib
