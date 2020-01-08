#ifndef __CAN_BLUEPILL__
#define __CAN_BLUEPILL__

#include "src/motor/dji_motor_tx_handler.hpp"
#include "can_rx_listener.hpp" 
#include "src/communication/can/can_rx_handler.hpp"
#include <stdio.h>

namespace aruwlib
{

    namespace can
    {

        /** This class extends the CanRxListner.
         * It has functionality to store and send can messages from CAN1 with valid ids
         * It can pack a can message with 4 ints
         * Further it allows for recieving and storing a valid can message, i.e. a message
         * with valid id
         */
        class CanBluepill: public CanRxListner {
            public:
            
            /** Creates a can object with given id 
             * Calls attachReceiveHandler to add to list of on recieve functions
             */
            CanBluepill(uint32_t id) : CanRxListner(static_cast<uint32_t>(id), CanBus::CAN_BUS1)
            {
                mcbMessage.setExtended(false); 
            } 
            
            /** This function recives a can message and if the id is valid
            * stores the message in the global can message
            */  
            void processMessage(const modm::can::Message& message); //rx

            /** Given an id, length and four integers, 
             * packet that into can message and send it 
             */ 
            void transferMessage(modm::can::Message can, int len, int16_t val1, int16_t val2 , int16_t val3 , int16_t val4, int can_channel); 
            
            modm::can::Message can;

            modm::can::Message mcbMessage; 

            private:
            /** Stores data into the global can message 
             */ 
            void storeMessage(const modm::can::Message& message);

            /** Packs the msg.bug[] with the given int byte by byte in little endian
             */ 
            void serializeCanBluePillSendData(int16_t ints_to_send[4], modm::can::Message* message);
        };

    }

}

#endif
