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

#### KeyToggle.hpp
```cpp
/**
 *  PWM modm interface
 *  A simple API that allows users to easily interact with 
 *  modm's PWM functionality. This implementation contains 
 *  a wrapper class on top of a Timer GeneralPurposeTimer
 *  class.
 */

#include <rm-dev-board-a/board.hpp>
#include "src/communication/remote.hpp"

//struct containing the key, if it is enabled, toggled, (state of toggle)
typedef struct{
	int16_t key;
	bool key_enabled;
	bool key_toggled;
	enum{
		NOT_PRESSED = 0,
		PRESSED = 1,
		RELEASED = 3,
		PRESSED_TO_UNTOGGLE = 4,
	} key_state;
} toggle_key_t;

//returns if the key is enabled and toggled on
bool key_toggled(uint16_t key);

//returns the toggled key
toggle_key_t get_toggle(int16_t key);

//puts all keys in a non-toggled mode
void zero_all_toggle_keys();

//initializes all states
void init_toggle_key(int16_t key);

//initializes based on robot type
void init_desired_keys();

//actually toggles the key on the computer's side of things, depending on the data given
void key_toggle_handler(toggle_key_t* key);

//calls on key toggle handler which then toggles the key to a given mode.
//will be called in keyToggleHandler
void toggle_handler();

//KeyToggle.hpp
