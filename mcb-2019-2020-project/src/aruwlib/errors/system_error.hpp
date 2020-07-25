#ifndef __SYSTEM_ERROR_HPP__
#define __SYSTEM_ERROR_HPP__

#include <string>

namespace aruwlib
{
namespace errors
{
static const uint8_t ERROR_LOCATION_SIZE = 8;

///< Location of errors; subject to change.
enum Location : uint8_t
{
    CAN_RX = 0,
    MOTOR_CONTROL,
    MPU6500,
    DJI_SERIAL,
    COMMAND_SCHEDULER,
    SUBSYSTEM,
    CONTROLLER_MAPPER,
    TURRET,
    SERVO,
    LOCATION_AMOUNT,
};

class SystemError
{
public:
    /**
     * Default constructs the SystemError.
     */
    SystemError();

    /**
     * Creates a SystemError with the passed in description, line number, file
     * name, and location.
     */
    SystemError(const char *desc, int line, const char *file, Location l);

    /**
     * @return `true` if e1 and e2 have identical elements, `false` otherwise.
     *      `char *` comparison is done using `strcmp` (as opposed to comparing
     *      the pointer itself).
     */
    friend bool operator==(const SystemError &e1, const SystemError &e2);

    int getLineNumber() const;

    const char *getDescription() const;

    const char *getFilename() const;

    Location getLocation() const;

private:
    int lineNumber;

    const char *description;

    const char *filename;

    Location location;
};

}  // namespace errors

}  // namespace aruwlib

#endif
