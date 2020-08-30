#ifndef GRABBER_SUBSYSTEM_HPP_
#define GRABBER_SUBSYSTEM_HPP_

#include <aruwlib/communication/gpio/digital.hpp>
#include <aruwlib/control/subsystem.hpp>

namespace aruwsrc
{
namespace engineer
{
/**
 * This is a subsystem code for Engineer grabber mechanism.
 * The grabber will be actuated by a single solenoid that
 * controls two pneumatic pistons.
 */
template <typename Drivers> class GrabberSubsystem : public aruwlib::control::Subsystem<Drivers>
{
public:
    explicit GrabberSubsystem(aruwlib::gpio::Digital::OutputPin pin)
        : pin(pin),
          isGrabberSqueezed(false)
    {
    }

    void setSqueezed(bool isGrabberSqueezed)
    {
        Drivers::digital.set(pin, isGrabberSqueezed);
        this->isGrabberSqueezed = isGrabberSqueezed;
    }

    bool isSqueezed() const { return isGrabberSqueezed; }

private:
    aruwlib::gpio::Digital::OutputPin pin;

    bool isGrabberSqueezed;
};  // GrabberSubsystem

}  // namespace engineer

}  // namespace aruwsrc

#endif  // GRABBER_SUBSYSTEM_HPP_
