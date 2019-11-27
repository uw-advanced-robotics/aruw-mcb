# Toggling of Keys Implementation Plan

## High Level 

Supposed to keep track of the state of the key. i.e. whether it is toggled on or just pressed on.
it can also be toggled and off.

For Example:
You can have a 'W' key that is pressed to move the bot forward, or you can just toggle it on. 
This is ont the optimal method, but imagine toggling on and off the shooting ability

## Implementation Steps

The steps used to implement this:
1. Research ability to keep track of states. 
2. look at last year's stuff to see if they match up.
3. Implement the struct or other method to keep track of state.
5. test on keyboard

## Code Plan

#### KeyStateToggle.hpp
```cpp

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

