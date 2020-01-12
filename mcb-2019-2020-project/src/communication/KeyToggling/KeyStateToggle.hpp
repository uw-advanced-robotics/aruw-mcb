#ifndef __KEY_STATE_TOGGLE_HPP__
#define __KEY_STATE_TOGGLE_HPP__

#include "src/aruwlib/communication/remote.hpp"

class KeyStateToggle{
    private:
        bool currentToggleState;//refers to on or off
        bool prevState; //refers to toggled or not
        aruwlib::Remote::Key currKey;


    public:
        //iniiializes all fields to their respective amounts
        //class constructor
        KeyStateToggle(aruwlib::Remote::Key key);

        //actually toggles the key on the computer's side of things
        void KeyToggleHandler();

        //will return true if the object's key is toggled
        bool keyToggled() const;

};
#endif //KeyStateToggle.hpp
