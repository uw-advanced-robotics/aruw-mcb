#include "can_bluepill.hpp" 
#include "src/communication/can/can_rx_handler.hpp"

namespace aruwlib
{

    namespace can
    {

        void CanBluepill::processMessage(const modm::can::Message& message) { 
            if (message.identifier == canIdentifier) {
                storeMessage(message); 
            }
        }

        void CanBluepill::transferMessage(modm::can::Message can, int length, int16_t val1, int16_t val2 , int16_t val3 , int16_t val4, bool can1Test) {
            modm::can::Message message;
            message.identifier = can.identifier;
            message.length = length;
        
            packArray(val1, &message);
            packArray(val2, &message);
            packArray(val3, &message);
            packArray(val4, &message);

            message.setExtended(false); 

            if (can1Test) {
                Can1::sendMessage(message);
            }
            else 
            {
                Can2::sendMessage(message); 
            }
        
        }
                
        void CanBluepill::printCanMsg() {
            // printf("MB: ");
            // printf(mcbMessage); 

            // printf("ID: 0x ");
            // printf(mcbMessage.identifier, HEX); // how do we do this in cpp

            // printf("EXT: ");
            // printf(mcbMessage.flags); 

            // printf("LEN: ");
            // printf(mcbMessage.length);
                
            // printf("DATA: ");
            //     for (int16_t i = 0; i < mcbMessage.length; i+=2) {
            //         int16_t x = mcbMessage.data[i];
            //         x += mcbMessage.data[i + 1] << 8;
            //         printf(x);
            //         printf(" ");
            //     }
            // printf("");
        }

        void CanBluepill::storeMessage(const modm::can::Message& message) {
            // mcbMessage.mb = message.mb; 
            mcbMessage.identifier = message.identifier; 
            mcbMessage.flags = message.flags; 
            mcbMessage.length = message.length; 
            
            memcpy(mcbMessage.data, message.data, sizeof(message.data));
        }

        void CanBluepill::packArray(int16_t x, modm::can::Message* message) { // fix!!!!!! arhghghghgh
            char byte;
            for (int16_t i = message->length; i >= 0; i--) {
                byte = x >> (i * 8);
                message->data[i] = byte & 0xFF;
            }
        }

    }

}
