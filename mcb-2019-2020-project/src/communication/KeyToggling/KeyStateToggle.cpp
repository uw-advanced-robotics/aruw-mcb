#include "KeyStateToggle.hpp"

KeyStateToggle::KeyStateToggle(aruwlib::Remote::Key key){
    currKey = key;
    currentToggleState = false;
    PrevState = false;
}

void KeyStateToggle::KeyToggleHandler(){
    bool input = aruwlib::Remote::keyPressed(currKey);
    // only when input is true
    if (input && PrevState != input) {
        currentToggleState = !currentToggleState;
    }
    PrevState = input;
}

bool KeyStateToggle::keyToggled() const{
    return PrevState;
}
