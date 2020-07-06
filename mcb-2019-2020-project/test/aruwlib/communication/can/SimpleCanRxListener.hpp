#ifndef SIMPLE_CAN_RXhLISTENER_HPP_
#define SIMPLE_CAN_RXhLISTENER_HPP_

#include <aruwlib/communication/can/can_rx_listener.hpp>

/**
 * A CanRxListener that doesn't do anything except increment a counter
 * when processMessage was called and stores the previous message.
 *
 * Doesn't remove itself from the CanRxHandler on destruction.
 */
class SimpleCanRxListener : public aruwlib::can::CanRxListener
{
public:
    SimpleCanRxListener(uint32_t id, aruwlib::can::CanBus cB)
        : CanRxListener(id, cB),
          processMessageCount(0),
          msg()
    {
    }

    virtual void processMessage(const modm::can::Message& message) override
    {
        processMessageCount++;
        msg = message;
    }

    int getProcessMessageCount() const { return processMessageCount; }

    const modm::can::Message& getPrevMessage() const { return msg; }

private:
    int processMessageCount;
    modm::can::Message msg;
};

#endif  // SIMPLE_CAN_RX_LISTENER_HPP_
