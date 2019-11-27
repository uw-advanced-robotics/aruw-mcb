#include "can_bluepill.hpp" 
#include "src/communication/can/can_rx_handler.hpp"

namespace aruwlib
{

    namespace can
    {

        public: 
        void processMessage(const modm::can::Message& message) { 
            if (message.identifier == canIdentifier) {
                storeMessage(message); 
            }
        }

        void transferMessage(modm::can::Message can, int length, int16_t val1, int16_t val2 , int16_t val3 , int16_t val4, bool can1Test) {//tx
            modm::can::Message& message;
            message.identifier = can.identifier;
            message.length = length;
        
            packArray(val1,2,0,&message);
            packArray(val2,2,2,&message);
            packArray(val3,2,4,&message);
            packArray(val4,2,6,&message);

            if (can1Test) {
                Can1::sendMessage(message);
            }
            else {
                Can2::sendMessage(message); 
            }
        
        }
                
        void printCanMsg() {
            printf("MB: ");
            printf(mcbMessage.mb); 

            printf("ID: 0x ");
            printf(mcbMessage.identifier, HEX); 

            printf("EXT: ");
            printf(mcbMessage.flags); 

            printf("LEN: ");
            printf(mcbMessage.length);
                
            printf("DATA: ");
                for (int16_t i = 0; i < mcbMessage.length; i+=2) {
                    int16_t x = mcbMessage.data[i];
                    x += mcbMessage.data[i + 1] << 8;
                    printf(x);
                    printf(" ");
                }
            printf("");
        }

        private:
        void storeMessage(const modm::can::Message& message) {
            mcbMessage.mb = message.mb; 
            mcbMessage.identifier = message.identifier; 
            mcbMessage.flags = message.flags; 
            mcbMessage.length = message.length; 
            
            memcpy(mcbMessage.data, message.data, sizeof(message.data));
        }


        void packArray(int16_t x, int byteSize, int startingIndex) {
            char byte;
            for (int16_t i = byteSize - 1; i >= 0; i--) {
                byte = x >> (i * 8);
                message->data[startingIndex + i] = byte & 0xFF;
            }
        }

    }

}
