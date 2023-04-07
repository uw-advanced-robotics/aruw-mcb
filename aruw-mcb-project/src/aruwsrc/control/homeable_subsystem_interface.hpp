#ifndef HOMEABLE_SUBSYSTEM_INTERFACE_HPP_
#define HOMEABLE_SUBSYSTEM_INTERFACE_HPP_

#include "tap/control/subsystem.hpp"

namespace aruwsrc::control
{
class HomeableSubsystemInterface : public tap::control::Subsystem
{
public:
    virtual void setMotorOutput(int32_t desiredOutput) = 0;
    virtual bool isStalled() const = 0;
    virtual void setLowerBound() = 0;
    virtual void setUpperBound() = 0;
};
}  // namespace aruwsrc::control

#endif