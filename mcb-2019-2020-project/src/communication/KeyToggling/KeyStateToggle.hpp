#include "src/communication/remote.hpp"

class KeyStateToggle{
    private:
        bool current_state;//refers to on or off
        bool is_pressed; //refers to toggled or not
        aruwlib::Remote::Key currKey;


    public:
        //iniiializes all fields to their respective amounts
        //class constructor
        KeyStateToggle(aruwlib::Remote::Key key);

        //actually toggles the key on the computer's side of things
        void KeyToggleHandler(bool input);

        //will return true if the object's key is toggled
        bool keyToggled() const;

        //returns if the key's state is on or off
        bool currState() const;

};
//KeyToggle.hpp
