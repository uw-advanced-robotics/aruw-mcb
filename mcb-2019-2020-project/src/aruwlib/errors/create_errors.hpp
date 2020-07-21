#ifndef __CREATE_ERRORS_HPP__
#define __CREATE_ERRORS_HPP__

#include "aruwlib/Drivers.hpp"

#include "system_error.hpp"

namespace aruwlib
{
namespace errors
{
/**
 * Example for how to create and add an error to the ErrorController:
 * ```
 * RAISE_ERROR("Error in DJI Serial", aruwlib::errors::Location::DJI_SERIAL);
 * ```
 * then call ErrorController::updateLedDisplay() to display the error via the LEDs
 */
#define RAISE_ERROR(desc, l)                                                   \
    do                                                                         \
    {                                                                          \
        aruwlib::errors::SystemError stringError(desc, __LINE__, __FILE__, l); \
        aruwlib::Drivers::errorController.addToErrorList(stringError);         \
    } while (0);

}  // namespace errors

}  // namespace aruwlib

#endif
