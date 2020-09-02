#include <modm/container/linked_list.hpp>

#include "aruwlib/architecture/timeout.hpp"
#include "aruwlib/communication/gpio/leds.hpp"
#include "aruwlib/Drivers.hpp"

#include "error_controller.hpp"

// Overall method to use when receiving errors

namespace aruwlib
{
namespace errors
{
    // add an error to list of errors
    void ErrorController::addToErrorList(SystemError error) {
        // only add error if it is not already added
        for (SystemError sysErr : errorList)
        {
            if (
                sysErr.getErrorType() == error.getErrorType()
                && sysErr.getLocation() == error.getLocation()
                && (sysErr.getDescription().compare(error.getDescription()) == 0)
                && (sysErr.getFilename().compare(error.getFilename()) == 0)
                && sysErr.getLineNumber() == error.getLineNumber()
            ) {
                return;  // the error is already added
            }
        }
        if (errorList.getSize() >= errorList.getMaxSize())
        {
            errorList.removeFront();  // remove the oldest element in the error list
        }
        errorList.append(error);
    }

    // Blink the list of errors in a loop on the board
    void ErrorController::update() {
        // there are no errors to display, default display
        if (errorList.getSize() == 0) {
            setLedError(0);
            Drivers::leds.set(aruwlib::gpio::Leds::LedPin::Green, true);
            return;
        }

        // change error every ERROR_ROTATE_TIME time increment
        if (prevLedErrorChangeWait.execute()) {
            prevLedErrorChangeWait.restart(ERROR_ROTATE_TIME);
            currentDisplayIndex = (currentDisplayIndex + 1) % errorList.getSize();
        }

        uint8_t displayNum = 0;
        if (getLedErrorCodeBits(errorList.get(currentDisplayIndex).getLocation(),
            errorList.get(currentDisplayIndex).getErrorType(), &displayNum)
        ) {
            setLedError(displayNum);
            Drivers::leds.set(aruwlib::gpio::Leds::LedPin::Green, true);
        } else {
            setLedError(0);
            Drivers::leds.set(aruwlib::gpio::Leds::LedPin::Green, false);
        }
    }

    bool ErrorController::getLedErrorCodeBits(
        Location location,
        ErrorType errorType,
        uint8_t* number
    ) {
        // Limit location and error type
        // Check to make sure they are within bounds

        // find the bit mask for the location
        uint8_t locationMask = static_cast<uint8_t>(~(0xffu << ERROR_LOCATION_SIZE));
        uint8_t errorTypeMask = static_cast<uint8_t>(~(0xffu << ERROR_TYPE_SIZE));

        uint8_t locationMasked = static_cast<uint8_t>(location) & locationMask;
        uint8_t errorTypeMasked = static_cast<uint8_t>(errorType) & errorTypeMask;

        // set another error if this error is outside of the range of the error handler
        if (locationMasked != location || errorTypeMasked != errorType) {
            return false;
        }

        // Combine location and error type
        *number = location << ERROR_LOCATION_SIZE | errorType;
        return true;
    }

    void ErrorController::setLedError(uint8_t binaryRep) {
        // Mask number and determine if it is a 0 or a 1
        // If it is a 1, the LED corresponding will blink
        for (int i = 0; i < 8; i++) {
            bool display = (binaryRep >> i) & 1;
            Drivers::leds.set(
                static_cast<aruwlib::gpio::Leds::LedPin>(i), display);
        }
    }
}  // namespace errors

}  // namespace aruwlib


