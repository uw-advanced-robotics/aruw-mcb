#ifndef CAN_RX_HANDLER_HPP_
#define CAN_RX_HANDLER_HPP_

#include <modm/architecture/interface/assert.h>

#include "aruwlib/errors/create_errors.hpp"
#include "aruwlib/motor/dji_motor_tx_handler.hpp"

#include "can_rx_listener.hpp"

namespace aruwlib
{
namespace can
{
/**
 * A handler that stores pointers to CanRxListener and that watches
 * CAN 1 and CAN 2 for messages. If messages are received, it checks
 * its internal maps for a `CanRxListener` that matches the message
 * identifier and CAN bus and calls the listener's `processMessage`
 * function.
 *
 * Interfaces with modm receive data from CAN1 and CAN2 buses.
 *
 * To use, extend CanRxListener class and create a method called
 * processMessage. Next, call the function attachReceiveHandler,
 * which will add the class you instantiated to a list of classes
 * that will be handled on receive. The class you created and
 * attached will be called by the pollCanData function
 * every time there is a message available that has the CAN identifier
 * matching the identifier specified in the CanRxListener constructor.
 *
 * For proper closed loop motor control, it is necessary to have the
 * pollCanData function be called at a very high frequency,
 * so call this in a high frequency thread.
 *
 * @see `CanRxListener` for information about how to properly create
 *      add a listener to the handler.
 * @see `Can` for modm CAN wrapper functions.
 */
template <typename Drivers>
class CanRxHandler
{
public:
    CanRxHandler() = default;

    ///< Delete copy constructor.
    CanRxHandler(const CanRxHandler&) = delete;

    ///< Delete operator=.
    CanRxHandler& operator=(const CanRxHandler& other) = delete;

    /**
     * Call this function to add a CanRxListener to the list of CanRxListener's
     * that are referenced when a new CAN message is received.
     *
     * @note do not call this function in an object that is globally (i.e. not on
     *      that stack or heap) constructed. The map that the handler uses to
     *      store listeners may or not be properly allocated if you do and
     *      undefined behavior will follow.
     * @note if you attempt to add a listener with an identifier identical to
     *      something already in the `CanRxHandler`, an error is thrown and
     *      the handler does not add the listener.
     * @see `CanRxListener`
     * @param[in] listener the listener to be attached ot the handler.
     * @return true if listener successfully added, false otherwise.
     */
    void attachReceiveHandler(CanRxListener<Drivers>* listener)
    {
        if (listener->canBus == can::CanBus::CAN_BUS1)
        {
            attachReceiveHandler(listener, messageHandlerStoreCan1, MAX_RECEIVE_UNIQUE_HEADER_CAN1);
        }
        else
        {
            attachReceiveHandler(listener, messageHandlerStoreCan2, MAX_RECEIVE_UNIQUE_HEADER_CAN2);
        }
    }

    /**
     * Function handles receiving messages and calling the appropriate
     * processMessage function given the CAN bus and can identifier.
     *
     * @attention you should call this function as frequently as you receive
     *      messages if you want to receive the most up to date messages.
     *      modm's IQR puts CAN messages in a queue, and this function
     *      clears out the queue once it is called.
     */
    void pollCanData()
    {
        modm::can::Message rxMessage;
        // handle incoming CAN 1 messages
        if (Drivers::can.getMessage(CanBus::CAN_BUS1, &rxMessage))
        {
            processReceivedCanData(
                rxMessage,
                messageHandlerStoreCan1,
                MAX_RECEIVE_UNIQUE_HEADER_CAN1);
        }
        // handle incoming CAN 2 messages
        if (Drivers::can.getMessage(CanBus::CAN_BUS2, &rxMessage))
        {
            processReceivedCanData(
                rxMessage,
                messageHandlerStoreCan2,
                MAX_RECEIVE_UNIQUE_HEADER_CAN2);
        }
    }

    /**
     * Removes the passed in `CanRxListener` from the `CanRxHandler`. If the
     * listener isn't in the handler, the
     */
    void removeReceiveHandler(const CanRxListener<Drivers>& rxListener)
    {
        if (rxListener.canBus == CanBus::CAN_BUS1)
        {
            removeReceiveHandler(
                rxListener,
                messageHandlerStoreCan1,
                MAX_RECEIVE_UNIQUE_HEADER_CAN1);
        }
        else
        {
            removeReceiveHandler(
                rxListener,
                messageHandlerStoreCan2,
                MAX_RECEIVE_UNIQUE_HEADER_CAN2);
        }
    }

private:
    static const int MAX_RECEIVE_UNIQUE_HEADER_CAN1 = 8;
    static const int MAX_RECEIVE_UNIQUE_HEADER_CAN2 = 8;
    static const int LOWEST_RECEIVE_ID = 0x201;

    /**
     * Stores pointers to the `CanRxListeners` for CAN 1, referenced when
     * a new message is received.
     */
    CanRxListener<Drivers>* messageHandlerStoreCan1[MAX_RECEIVE_UNIQUE_HEADER_CAN1];

    /**
     * Stores pointers to the `CanRxListeners` for CAN 2, referenced when
     * a new message is received.
     */
    CanRxListener<Drivers>* messageHandlerStoreCan2[MAX_RECEIVE_UNIQUE_HEADER_CAN2];

    /**
     * Does the work of the public `attachReceiveHandler` function, but
     * handles a particular CAN bus.
     *
     * @see `attachReceiveHandler`
     */
    void attachReceiveHandler(
        CanRxListener<Drivers>* CanRxHndl,
        CanRxListener<Drivers>** messageHandlerStore,
        int messageHandlerStoreSize)
    {
        int32_t id = DJI_MOTOR_NORMALIZED_ID(CanRxHndl->canIdentifier);
        bool receiveInterfaceOverloaded = messageHandlerStore[id] != nullptr;
        bool receiveAttachSuccess =
            !receiveInterfaceOverloaded || (id >= 0 && id < messageHandlerStoreSize);
        modm_assert(receiveAttachSuccess, "can1", "receive init", "overloading", 1);

        messageHandlerStore[id] = CanRxHndl;
    }

    inline void processReceivedCanData(
        const modm::can::Message& rxMessage,
        CanRxListener<Drivers>** messageHandlerStore,
        int messageHandlerStoreSize)
    {
        int32_t handlerStoreId = DJI_MOTOR_NORMALIZED_ID(rxMessage.getIdentifier());
        if (handlerStoreId >= 0 && handlerStoreId < messageHandlerStoreSize)
        {
            if (messageHandlerStore[handlerStoreId] != nullptr)
            {
                messageHandlerStore[handlerStoreId]->processMessage(rxMessage);
            }
        }
        else
        {
            RAISE_ERROR(
                "Invalid can id received - not between 0x200 and 0x208",
                aruwlib::errors::Location::CAN_RX,
                aruwlib::errors::ErrorType::MOTOR_ID_OUT_OF_BOUNDS);
        }
    }

    void removeReceiveHandler(
        const CanRxListener<Drivers>& listener,
        CanRxListener<Drivers>** messageHandlerStore,
        int messageHandlerStoreSize)
    {
        int id = DJI_MOTOR_NORMALIZED_ID(listener.canIdentifier);
        if (id < 0 || id >= messageHandlerStoreSize)
        {
            RAISE_ERROR(
                "index out of bounds",
                aruwlib::errors::CAN_RX,
                aruwlib::errors::INVALID_REMOVE);
            return;
        }
        messageHandlerStore[id] = nullptr;
    }
};  // class CanRxHandler

}  // namespace can

}  // namespace aruwlib

#endif  // CAN_RX_HANDLER_HPP_
