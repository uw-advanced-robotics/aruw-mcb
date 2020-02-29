// Subsystem for grabber mechanism

#include "src/aruwsrc/control/grabber_subsystem.hpp"

#include <rm-dev-board-a/board.hpp>

namespace aruwsrc
{

namespace control
{
    void GrabberSubsystem::refresh(void) {
        grabberDigitalOutPin::set(isGrabberSqueezed); 
    }

    void GrabberSubsystem::setSqueezed(bool isGrabberSqueezed) {
        this->isGrabberSqueezed = isGrabberSqueezed; 
        grabberDigitalOutPin::set(isGrabberSqueezed);
    }
}  // namespace control

}  // namespace aruwsrc
