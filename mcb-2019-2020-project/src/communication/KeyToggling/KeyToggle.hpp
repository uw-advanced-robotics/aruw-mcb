#include <rm-dev-board-a/board.hpp>
#include "src/communication/remote.hpp"

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
void toggle_handler();