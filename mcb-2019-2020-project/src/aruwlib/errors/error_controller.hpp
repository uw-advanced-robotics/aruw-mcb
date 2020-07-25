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
 * The ErrorController stores the errors that are currently active and displays errors
 * via the MCB's LEDs.
 *
 * Use the `RAISE_ERROR` macro to add errors to the main ErrorController.
 *
 * LED blink Protocol description:
 * - The 8 LEDs on the MCB are used to indicate the location of an error. LED A is the LSB
 * and LED H is the MSB.<br>
 * - The other green LED (next to the red LED) comes on when you have added an error that
 * is invalid. The red LED is not used by the ErrorController.
 * - By default, LEDs A-H are always off if no errors are detected.
 */
class ErrorController
{
public:
    /**
     * Constrcuts an ErrorController with a display time for each error specified
     * by `ERROR_ROTATE_TIME`.
     */
    ErrorController() : prevLedErrorChangeWait(ERROR_ROTATE_TIME), currentDisplayIndex(0) {}

    /**
     * Default copy constructor.
     */
    ErrorController(const ErrorController&) = default;

    /**
     * Default equals operator.
     */
    ErrorController& operator=(const ErrorController&) = default;

    /**
     * Adds the passed in error to the ErrorController if no identical errors are already in
     * the ErrorController.
     *
     * @param[in] error The SystemError to add to the ErrorController.
     */
    void addToErrorList(const SystemError& error);

    /**
     * Updates the LED display. Cycles through the SystemErrors in the queue of errors,
     * switching to a new error every `ERROR_ROTATE_TIME` (5 seconds).
     */
    void updateLedDisplay();

    int getErrorListSize() const { return errorList.getSize(); }

    /**
     * Returns the SystemError at the specified index, zero index from the start
     * of the error array.
     */
    const SystemError* getSystemError(int index) const;

    /**
     * Removes the SystemError that matches the passed in SystemError.
     *
     * @param[in] error The SystemError to remove. The SystemError is considered
     *      matching if the location, description, filename, and line number matches.
     * @return `true` if the error was in the ErrorController and was successfully
     *      removed, `false` otherwise.
     */
    bool removeSystemError(const SystemError& error);

private:
    static const int ERROR_ROTATE_TIME = 5000;

    static const unsigned ERROR_LIST_MAX_SIZE = 16;

    modm::BoundedDeque<SystemError, ERROR_LIST_MAX_SIZE> errorList;

    aruwlib::arch::MilliTimeout prevLedErrorChangeWait;

    int currentDisplayIndex;

    /**
     * Displays the `binaryRep` with the LEDs A-H, with LED A as the LSB and LED H as the MSB.
     */
    void displayBinaryNumberWithLeds(uint8_t binaryRep);
};

}  // namespace errors

}  // namespace aruwlib

#endif
