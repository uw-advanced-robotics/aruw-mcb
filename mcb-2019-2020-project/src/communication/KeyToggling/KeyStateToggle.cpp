#include "KeyStateToggle.hpp"

namespace aruwlib
{

namespace communication
{

KeyStateToggle::KeyStateToggle(aruwlib::Remote::Key key)
    : currKey(key)
    , prevState(false)
    , currentToggleState(false)
    {}  // empty compound statement

void KeyStateToggle::KeyToggleHandler(){
    bool input = aruwlib::Remote::keyPressed(currKey);
    // ensures a change in the toggle state only when the input is true i.e the key is pressed
    // and the previous input or toggled state does not equal input i.e. only swaps the state when
    // the input given is different
    if (input && prevState != input) {
        currentToggleState = !currentToggleState;
    }
    prevState = input;
}

bool KeyStateToggle::keyToggled() const{
    return currentToggleState;
}

}  // namespace communication

}  // namespace aruwlib
