#include "system_error.hpp"

#include <cstring>

namespace aruwlib
{
namespace errors
{
SystemError::SystemError()
    : lineNumber(0),
      description("default"),
      filename("none"),
      location(LOCATION_AMOUNT)
{
    static_assert(
        LOCATION_AMOUNT <= ERROR_LOCATION_SIZE * ERROR_LOCATION_SIZE,
        "You have declared too many locations!");
}

SystemError::SystemError(const char *desc, int line, const char *file, Location l)
    : lineNumber(line),
      description(desc),
      filename(file),
      location(l)
{
    static_assert(
        LOCATION_AMOUNT <= ERROR_LOCATION_SIZE * ERROR_LOCATION_SIZE,
        "You have declared too many locations!");
}

bool operator==(const SystemError &e1, const SystemError &e2)
{
    return e1.lineNumber == e2.lineNumber && !strcmp(e1.description, e2.description) &&
           !strcmp(e1.filename, e2.filename) && e1.location == e2.location;
}

int SystemError::getLineNumber() const { return lineNumber; }

const char *SystemError::getDescription() const { return description; }

const char *SystemError::getFilename() const { return filename; }

Location SystemError::getLocation() const { return location; }
}  // namespace errors

}  // namespace aruwlib
