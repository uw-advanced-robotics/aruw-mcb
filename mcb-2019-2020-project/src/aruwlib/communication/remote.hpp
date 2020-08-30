#ifndef REMOTE_HPP_
#define REMOTE_HPP_

#include <cstdint>

#ifndef ENV_SIMULATOR
#include <modm/platform.hpp>
#endif

#include "aruwlib/algorithms/math_user_utils.hpp"
#include "aruwlib/architecture/clock.hpp"
#include "aruwlib/communication/serial/uart.hpp"

namespace aruwlib
{
namespace remote
{
/**
 * Specifies a particular joystick.
 */
enum class Channel
{
    RIGHT_HORIZONTAL,
    RIGHT_VERTICAL,
    LEFT_HORIZONTAL,
    LEFT_VERTICAL
};

/**
 * Specifies a particular switch.
 */
enum class Switch
{
    LEFT_SWITCH,
    RIGHT_SWITCH
};

/**
 * Different switch orientations.
 */
enum class SwitchState
{
    UNKNOWN,
    UP,
    MID,
    DOWN
};

/**
 * A list of the particular keys to interact with, in bit order.
 */
enum class Key
{
    W = 0,
    S,
    A,
    D,
    SHIFT,
    CTRL,
    Q,
    E,
    R,
    F,
    G,
    Z,
    X,
    C,
    V,
    B
};
}  // namespace remote

/**
 * A unique UART handler that uses timing in leu of DBUS communication (modm does not
 * support DBUS) to interact with the DR16 receiver.
 */
template <typename Drivers> class Remote
{
public:
    Remote() = default;
    Remote(const Remote &) = delete;
    Remote &operator=(const Remote &) = delete;

    /**
     * Enables and initializes `Uart::Uart1` communication.
     */
    void initialize()
    {
        Drivers::uart.template init<serial::Uart::Uart1, 100000, serial::Uart::Parity::Even>();
    }

    /**
     * Reads/parses the current buffer and updates the current remote info state
     * and `CommandMapper` state.
     */
    void read()
    {
        // Check disconnect timeout
        if (aruwlib::arch::clock::getTimeMilliseconds() - lastRead > REMOTE_DISCONNECT_TIMEOUT)
        {
            connected = false;  // Remote no longer connected
            reset();            // Reset current remote values
        }
        uint8_t data;  // Next byte to be read
        // Read next byte if available and more needed for the current packet
        while (Drivers::uart.read(serial::Uart::UartPort::Uart1, &data) &&
               currentBufferIndex < REMOTE_BUF_LEN)
        {
            rxBuffer[currentBufferIndex] = data;
            currentBufferIndex++;
            lastRead = aruwlib::arch::clock::getTimeMilliseconds();
        }
        // Check read timeout
        if (aruwlib::arch::clock::getTimeMilliseconds() - lastRead > REMOTE_READ_TIMEOUT)
        {
            clearRxBuffer();
        }
        // Parse buffer if all 18 bytes are read
        if (currentBufferIndex >= REMOTE_BUF_LEN)
        {
            connected = true;
            parseBuffer();
            clearRxBuffer();
        }
    }

    /**
     * @return `true` if the remote is connected, `false` otherwise.
     * @note A timer is used to determine if the remote is disconnected, so expect a
     *      second or so of delay from disconnecting the remote to this function saying
     *      the remote is disconnected.
     */
    bool isConnected() const { return connected; }

    /**
     * @return The value of the given channel, between [-1, 1].
     */
    float getChannel(remote::Channel ch) const
    {
        switch (ch)
        {
            case remote::Channel::RIGHT_HORIZONTAL:
                return remote.rightHorizontal / STICK_MAX_VALUE;
            case remote::Channel::RIGHT_VERTICAL:
                return remote.rightVertical / STICK_MAX_VALUE;
            case remote::Channel::LEFT_HORIZONTAL:
                return remote.leftHorizontal / STICK_MAX_VALUE;
            case remote::Channel::LEFT_VERTICAL:
                return remote.leftVertical / STICK_MAX_VALUE;
        }
        return 0;
    }

    /**
     * @return The state of the given switch.
     */
    remote::SwitchState getSwitch(remote::Switch sw) const
    {
        switch (sw)
        {
            case remote::Switch::LEFT_SWITCH:
                return remote.leftSwitch;
            case remote::Switch::RIGHT_SWITCH:
                return remote.rightSwitch;
        }
        return remote::SwitchState::UNKNOWN;
    }

    /**
     * @return The current mouse x value.
     */
    int16_t getMouseX() const { return remote.mouse.x; }

    /**
     * @return The current mouse y value.
     */
    int16_t getMouseY() const { return remote.mouse.y; }

    /**
     * @return The current mouse z value.
     */
    int16_t getMouseZ() const { return remote.mouse.z; }

    /**
     * @return The current mouse l value.
     */
    bool getMouseL() const { return remote.mouse.l; }

    /**
     * @return The current mouse r value.
     */
    bool getMouseR() const { return remote.mouse.r; }

    /**
     * @return `true` if the given `key` is pressed, `false` otherwise.
     */
    bool keyPressed(remote::Key key) const { return (remote.key & (1 << (uint8_t)key)) != 0; }

    /**
     * @return the value of the wheel, between `[-STICK_MAX_VALUE, STICK_MAX_VALUE]`.
     */
    int16_t getWheel() const { return remote.wheel; }

    /**
     * @return the number of times remote info has been received.
     */
    uint32_t getUpdateCounter() const { return remote.updateCounter; }

private:
    static const int REMOTE_BUF_LEN = 18;              ///< Length of the remote recieve buffer.
    static const int REMOTE_READ_TIMEOUT = 6;          ///< Timeout delay between valid packets.
    static const int REMOTE_DISCONNECT_TIMEOUT = 100;  ///< Timeout delay for remote disconnect.
    static const int REMOTE_INT_PRI = 12;              ///< Interrupt priority.
    static constexpr float STICK_MAX_VALUE = 660.0f;   ///< Max value received by one of the sticks.

    ///< The current remote information
    struct RemoteInfo
    {
        uint32_t updateCounter = 0;
        int16_t rightHorizontal = 0;
        int16_t rightVertical = 0;
        int16_t leftHorizontal = 0;
        int16_t leftVertical = 0;
        remote::SwitchState leftSwitch = remote::SwitchState::UNKNOWN;
        remote::SwitchState rightSwitch = remote::SwitchState::UNKNOWN;
        struct
        {  ///< Mouse information
            int16_t x = 0;
            int16_t y = 0;
            int16_t z = 0;
            bool l = false;
            bool r = false;
        } mouse;
        uint16_t key = 0;   ///< Keyboard information
        int16_t wheel = 0;  ///< Remote wheel information
    };

    RemoteInfo remote;

    ///< Remote connection state.
    bool connected = false;

    ///< UART recieve buffer.
    uint8_t rxBuffer[REMOTE_BUF_LEN]{0};

    ///< Timestamp when last byte was read (milliseconds).
    uint32_t lastRead = 0;

    ///< Current count of bytes read.
    uint8_t currentBufferIndex = 0;

    ///< Parses the current rxBuffer.
    void parseBuffer()
    {
        // values implemented by shifting bits across based on the dr16
        // values documentation and code created last year
        remote.rightHorizontal = (rxBuffer[0] | rxBuffer[1] << 8) & 0x07FF;
        remote.rightHorizontal -= 1024;
        remote.rightVertical = (rxBuffer[1] >> 3 | rxBuffer[2] << 5) & 0x07FF;
        remote.rightVertical -= 1024;
        remote.leftHorizontal = (rxBuffer[2] >> 6 | rxBuffer[3] << 2 | rxBuffer[4] << 10) & 0x07FF;
        remote.leftHorizontal -= 1024;
        remote.leftVertical = (rxBuffer[4] >> 1 | rxBuffer[5] << 7) & 0x07FF;
        remote.leftVertical -= 1024;
        // the first 6 bytes refer to the remote channel values

        // switches on the dji remote - their input is registered
        switch (((rxBuffer[5] >> 4) & 0x000C) >> 2)
        {
            case 1:
                remote.leftSwitch = remote::SwitchState::UP;
                break;
            case 3:
                remote.leftSwitch = remote::SwitchState::MID;
                break;
            case 2:
                remote.leftSwitch = remote::SwitchState::DOWN;
                break;
            default:
                remote.leftSwitch = remote::SwitchState::UNKNOWN;
                break;
        }

        switch ((rxBuffer[5] >> 4) & 0x003)
        {
            case 1:
                remote.rightSwitch = remote::SwitchState::UP;
                break;
            case 3:
                remote.rightSwitch = remote::SwitchState::MID;
                break;
            case 2:
                remote.rightSwitch = remote::SwitchState::DOWN;
                break;
            default:
                remote.rightSwitch = remote::SwitchState::UNKNOWN;
                break;
        }

        // remaining 12 bytes (based on the DBUS_BUF_LEN variable
        // being 18) use mouse and keyboard data
        // 660 is the max value from the remote, so gaining a higher
        // value would be impractical.
        // as such, the method returns null, exiting the method.
        if ((abs(remote.rightHorizontal) > 660) || (abs(remote.rightVertical) > 660) ||
            (abs(remote.leftHorizontal) > 660) || (abs(remote.leftVertical) > 660))
        {
            return;
        }

        // mouse input
        remote.mouse.x = rxBuffer[6] | (rxBuffer[7] << 8);    // x axis
        remote.mouse.y = rxBuffer[8] | (rxBuffer[9] << 8);    // y axis
        remote.mouse.z = rxBuffer[10] | (rxBuffer[11] << 8);  // z axis
        remote.mouse.l = static_cast<bool>(rxBuffer[12]);     // left button click
        remote.mouse.r = static_cast<bool>(rxBuffer[13]);     // right button click

        // keyboard capture
        remote.key = rxBuffer[14] | rxBuffer[15] << 8;
        // Remote wheel
        remote.wheel = (rxBuffer[16] | rxBuffer[17] << 8) - 1024;

        Drivers::commandMapper.handleKeyStateChange(
            remote.key,
            remote.leftSwitch,
            remote.rightSwitch);

        remote.updateCounter++;
    }

    ///< Clears the current rxBuffer.
    void clearRxBuffer()
    {
        // Reset bytes read counter
        currentBufferIndex = 0;
        // Clear remote rxBuffer
        for (int i = 0; i < REMOTE_BUF_LEN; i++)
        {
            rxBuffer[i] = 0;
        }
        // Clear Usart1 rxBuffer
        Drivers::uart.discardReceiveBuffer(serial::Uart::UartPort::Uart1);
    }

    ///< Resets the current remote info.
    void reset()
    {
        remote.rightHorizontal = 0;
        remote.rightVertical = 0;
        remote.leftHorizontal = 0;
        remote.leftVertical = 0;
        remote.leftSwitch = remote::SwitchState::UNKNOWN;
        remote.rightSwitch = remote::SwitchState::UNKNOWN;
        remote.mouse.x = 0;
        remote.mouse.y = 0;
        remote.mouse.z = 0;
        remote.mouse.l = 0;
        remote.mouse.r = 0;
        remote.key = 0;
        remote.wheel = 0;
        clearRxBuffer();
    }
};  // class Remote

}  // namespace aruwlib

#endif  // REMOTE_HPP_
