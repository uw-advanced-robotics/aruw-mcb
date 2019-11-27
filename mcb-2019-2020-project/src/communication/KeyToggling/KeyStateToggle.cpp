#include "KeyStateToggle.hpp"

KeyStateToggle::KeyStateToggle(aruwlib::Remote::Key key){
    currKey = key;
    current_state = false;
    is_pressed = false;
}

void KeyStateToggle::KeyToggleHandler(bool input){
    if(input != keyToggled()){
        is_pressed = !is_pressed;
        current_state != current_state;
    }
}

bool KeyStateToggle::keyToggled() const{
    return is_pressed;
}

bool KeyStateToggle::currState() const{
    return current_state;
}