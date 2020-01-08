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

        void CanBluepill::transferMessage(modm::can::Message can, int length, int16_t val1, int16_t val2 , int16_t val3 , int16_t val4, int can_channel) {
            modm::can::Message message;
            message.identifier = can.identifier;
            message.length = length;
            int16_t ints_to_send[4] = {val1, val2, val3, val4};
            serializeCanBluePillSendData(ints_to_send, &message);

            message.setExtended(false); 

            if (can_channel = 1) {
                Can1::sendMessage(message);
            }
            else // can_channel = 2
            {
                Can2::sendMessage(message); 
            }
        
        }

        void CanBluepill::storeMessage(const modm::can::Message& message) {
            // mcbMessage.mb = message.mb; 
            mcbMessage.identifier = message.identifier; 
            mcbMessage.flags = message.flags; 
            mcbMessage.length = message.length; 
            
            memcpy(mcbMessage.data, message.data, sizeof(message.data));
        }

        void CanBluepill::serializeCanBluePillSendData(int16_t ints_to_send[4], modm::can::Message* message) {
            for (int16_t i = 0; i < 4; i++) {
                message->data[2 * i + 1] = ints_to_send[i] >> 8;
                message->data[2 * i] = ints_to_send[i] & 0xFF;
            }
        }

    }

}
