#include "KeyToggle.hpp"

toggle_key_t toggle_keys[16];
toggle_key_t dummy_key;

toggle_key_t get_toggle(int16_t key){
    for(int i = 0; i < 16; i++){
        if(key & toggle_keys[i].key && toggle_keys[i].key_enabled){
            return toggle_keys[i];
        }
    }
    return dummy_key;
}

bool key_toggled(uint16_t key){
  return get_toggle(key).key_enabled && get_toggle(key).key_toggled;
}

void zero_all_toggle_keys(){
    for(int i = 0; i < 16; i++){
        toggle_keys[i].key_enabled = false;
        toggle_keys[i].key = ((uint16_t)0x01 << i);
        toggle_keys[i].key_state = toggle_key_t::NOT_PRESSED;
    }
    dummy_key.key_enabled = false;
    dummy_key.key = 0x00;
    dummy_key.key_state = toggle_key_t::NOT_PRESSED;
}

void init_toggle_key(int16_t key){
    for(int i = 0; i < 16; i++){
        if(!(toggle_keys[i].key ^ key)){
            toggle_keys[i].key_enabled = true;
        }
    }
}

void init_desired_keys(){
    zero_all_toggle_keys();
    #if defined(TARGET_SOLDIER)
    init_toggle_key(KEY_WIGGLE);
    init_toggle_key(KEY_HOPPER);
    init_toggle_key(KEY_LOW_CHASSIS_SPEED);
    #elif defined(TARGET_ENGINEER)
    init_toggle_key(KEY_DUMP);
    init_toggle_key(KEY_TOW);
  init_toggle_key(KEY_GRAB);
  init_toggle_key(KEY_GRAB_TGGL);
    init_toggle_key(KEY_LIFT_TGGL);
  #endif
}
    
void key_toggle_handler(toggle_key_t* key){ 
    switch(key->key_state){
        case toggle_key_t::NOT_PRESSED:
            key->key_toggled = false;
            //key->key_state = KEY_PRESSED(remote_dr16.computer.key, key->key) ? toggle_key_t::PRESSED : key->key_state;        
            break;
        case toggle_key_t::PRESSED:
            key->key_toggled = true;
            //key->key_state = !KEY_PRESSED(remote_dr16.computer.key, key->key) ? toggle_key_t::RELEASED : key->key_state;
            break;
        case toggle_key_t::RELEASED: 
            //key->key_state = KEY_PRESSED(remote_dr16.computer.key, key->key) ? toggle_key_t::PRESSED_TO_UNTOGGLE : key->key_state;
            break;
        case toggle_key_t::PRESSED_TO_UNTOGGLE:
            //key->key_state = !KEY_PRESSED(remote_dr16.computer.key, key->key) ? toggle_key_t::NOT_PRESSED : key->key_state;
    }
}
void toggle_handler(void){
    for(int i = 0; i < 16; i++){
        if(toggle_keys[i].key_enabled){
            key_toggle_handler(&toggle_keys[i]);
        }
    }
}
