#include "src/communication/remote.hpp"

class KeyStateToggle{
    public:
        typedef struct{
	        aruwlib::Remote::Key key;
	        bool key_enabled;
	        bool key_toggled;
	        enum{
		        NOT_PRESSED = 0,
		        PRESSED = 1,
		        RELEASED = 3,
		        PRESSED_TO_UNTOGGLE = 4,
	        } key_state;
        } toggle_key_t;

        //iniiializes all fields to their respective amounts
        //class constructor
        KeyStateToggle();

        //gets if the key is toggled
        bool getToggled(aruwlib::Remote::Key currKey);

        //actually toggles the key on the computer's side of things
        void KeyToggleHandler();

        //will return true if the object's key is toggled
        bool keyToggled();
};