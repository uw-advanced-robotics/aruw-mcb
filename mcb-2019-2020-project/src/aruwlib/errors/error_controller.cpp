#include "error_controller.hpp"

#include <string>

#include <modm/container/linked_list.hpp>

#include "aruwlib/Drivers.hpp"
#include "aruwlib/architecture/timeout.hpp"
#include "aruwlib/communication/gpio/leds.hpp"

namespace aruwlib
{
namespace errors
{
// add an error to list of errors
void ErrorController::addToErrorList(const SystemError &error)
{
    // only add error if it is not already added
    // Note that we are okay with comparing raw char pointers because an error generated
    // in our codebase use char pointers located in literals.
    for (SystemError sysErr : errorList)
    {
        if (sysErr.getLocation() == error.getLocation() &&
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

const SystemError *ErrorController::getSystemError(int index) const
{
    if (index < 0 || index >= static_cast<int>(errorList.getSize()))
    {
        return nullptr;
    }
    return &errorList.get(index);
}

bool ErrorController::removeSystemError(const SystemError &error)
{
    int size = errorList.getSize();
    int errorFoundIndex = -1;
    // We must walk through the list one index at a time.
    for (int i = 0; i < size; i++)
    {
        SystemError se = errorList.get(0);
        errorList.removeFront();
        if (errorFoundIndex == -1 && se == error)
        {
            errorFoundIndex = i;
        }
        else
        {
            errorList.append(se);
        }
    }
    // If the currentDisplayIndex is greater than or equal to the index we have removed, we must
    // decrease this value so it points to the same error in the queue.
    if (errorFoundIndex <= currentDisplayIndex && currentDisplayIndex != 0)
    {
        currentDisplayIndex--;
    }
    return errorFoundIndex != -1;
}

// Blink the list of errors in a loop on the board
void ErrorController::updateLedDisplay()
{
    // there are no errors to display, default display
    if (errorList.getSize() == 0)
    {
        displayBinaryNumberWithLeds(0);
        Drivers::leds.set(aruwlib::gpio::Leds::LedPin::Green, true);
        return;
    }

    // change error every ERROR_ROTATE_TIME time increment
    if (prevLedErrorChangeWait.execute())
    {
        prevLedErrorChangeWait.restart(ERROR_ROTATE_TIME);
        currentDisplayIndex = (currentDisplayIndex + 1) % errorList.getSize();
    }

    displayBinaryNumberWithLeds(
        static_cast<uint8_t>(errorList.get(currentDisplayIndex).getLocation()));
}

void ErrorController::displayBinaryNumberWithLeds(uint8_t binaryRep)
{
    // Mask number and determine if it is a 0 or a 1
    // If it is a 1, the LED corresponding will light up.
    for (int i = 0; i < 8; i++)
    {
        bool display = ((~binaryRep) >> i) & 1;
        Drivers::leds.set(static_cast<aruwlib::gpio::Leds::LedPin>(i), display);
    }
}
}  // namespace errors

}  // namespace aruwlib
