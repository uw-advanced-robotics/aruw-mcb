#include "src/communication/remote.hpp"

class KeyStateToggle{
    private:
        bool currentToggleState;//refers to on or off
        bool PrevState; //refers to toggled or not
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
//KeyToggle.hpp
