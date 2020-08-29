#ifndef ERROR_CONTROLLER_HPP
#define ERROR_CONTROLLER_HPP

#include <aruwlib/architecture/timeout.hpp>
#include <modm/container.hpp>

#include "system_error.hpp"

namespace aruwlib
{
namespace errors
{
/**
 * Protocol description:
 * The 8 leds on the mcb are used to indicate a location and error type. LEDs A-D transmit
 * the error location and E-H the error type.
 * For location, the LSB is D, and for error type, the LSB is H
 * The other green led (next to the red led) comes on
 * when you have added an invalid error. The red led is always on (not used). Default, leds
 * A-H are always off if no errors are detected
 */
template <typename Drivers> class ErrorController
{
public:
    ErrorController() : prevLedErrorChangeWait(ERROR_ROTATE_TIME) {}
    ErrorController(const ErrorController<Drivers>&) = delete;
    ErrorController& operator=(const ErrorController<Drivers>&) = delete;

    void addToErrorList(const SystemError& error)
    {
        // only add error if it is not already added
        // Note that we are okay with comparing raw char pointers because an error generated
        // in our codebase use char pointers located in literals.
        for (SystemError sysErr : errorList)
        {
            if (sysErr.getErrorType() == error.getErrorType() &&
                sysErr.getLocation() == error.getLocation() &&
                sysErr.getDescription() == error.getDescription() &&
                sysErr.getFilename() == error.getFilename() &&
                sysErr.getLineNumber() == error.getLineNumber())
            {
                return;  // the error is already added
            }
        }
        if (errorList.getSize() >= errorList.getMaxSize())
        {
            errorList.removeFront();  // remove the oldest element in the error list
        }
        errorList.append(error);
    }

    void update()
    {
        // there are no errors to display, default display
        if (errorList.getSize() == 0)
        {
            setLedError(0);
            Drivers::leds.set(aruwlib::gpio::Leds::LedPin::Green, true);
            return;
        }

        // change error every ERROR_ROTATE_TIME time increment
        if (prevLedErrorChangeWait.execute())
        {
            prevLedErrorChangeWait.restart(ERROR_ROTATE_TIME);
            currentDisplayIndex = (currentDisplayIndex + 1) % errorList.getSize();
        }

        uint8_t displayNum = 0;
        if (getLedErrorCodeBits(
                errorList.get(currentDisplayIndex).getLocation(),
                errorList.get(currentDisplayIndex).getErrorType(),
                &displayNum))
        {
            setLedError(displayNum);
            Drivers::leds.set(aruwlib::gpio::Leds::LedPin::Green, true);
        }
        else
        {
            setLedError(0);
            Drivers::leds.set(aruwlib::gpio::Leds::LedPin::Green, false);
        }
    }

private:
    static const int ERROR_ROTATE_TIME = 5000;

    static const unsigned ERROR_LIST_MAX_SIZE = 16;

    modm::BoundedDeque<SystemError, ERROR_LIST_MAX_SIZE> errorList;

    aruwlib::arch::MilliTimeout prevLedErrorChangeWait;

    int currentDisplayIndex = 0;

    bool getLedErrorCodeBits(Location location, ErrorType errorType, uint8_t* number)
    {
        // Limit location and error type
        // Check to make sure they are within bounds

        // find the bit mask for the location
        uint8_t locationMask = static_cast<uint8_t>(~(0xffu << ERROR_LOCATION_SIZE));
        uint8_t errorTypeMask = static_cast<uint8_t>(~(0xffu << ERROR_TYPE_SIZE));

        uint8_t locationMasked = static_cast<uint8_t>(location) & locationMask;
        uint8_t errorTypeMasked = static_cast<uint8_t>(errorType) & errorTypeMask;

        // set another error if this error is outside of the range of the error handler
        if (locationMasked != location || errorTypeMasked != errorType)
        {
            return false;
        }

        // Combine location and error type
        *number = location << ERROR_LOCATION_SIZE | errorType;
        return true;
    }

    void setLedError(uint8_t binaryRep)
    {
        // Mask number and determine if it is a 0 or a 1
        // If it is a 1, the LED corresponding will blink
        for (int i = 0; i < 8; i++)
        {
            bool display = (binaryRep >> i) & 1;
            Drivers::leds.set(static_cast<aruwlib::gpio::Leds::LedPin>(i), display);
        }
    }
};

}  // namespace errors

}  // namespace aruwlib

#endif
