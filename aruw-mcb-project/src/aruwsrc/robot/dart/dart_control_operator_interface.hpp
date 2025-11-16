#ifndef DART_CONTROL_OPERATOR_INTERFACE_HPP_
#define DART_CONTROL_OPERATOR_INTERFACE_HPP_

#include "aruwsrc/robot/control_operator_interface.hpp"

namespace aruwsrc::control::dart
{
class DartControlOperatorInterface : public ControlOperatorInterface
{
public:
    DartControlOperatorInterface(tap::Drivers* drivers)
        : ControlOperatorInterface(drivers),
          drivers(drivers)
    {
    }

    float getPullbackVelocity();

private:
    tap::Drivers* drivers;
};
}  // namespace aruwsrc::control::dart

#endif  // DART_CONTROL_OPERATOR_INTERFACE_HPP_