// Subsystem for grabber mechanism

#include "grabber_subsystem.hpp"

#include <rm-dev-board-a/board.hpp>

namespace aruwsrc
{

namespace engineer
{
    void GrabberSubsystem::refresh(void) {
        grabberDigitalOutPin::set(isGrabberSqueezed); 
    }

    void GrabberSubsystem::setSqueezed(bool isGrabberSqueezed) {
        this->isGrabberSqueezed = isGrabberSqueezed; 
        grabberDigitalOutPin::set(isGrabberSqueezed);
    }
}  // namespace engineer

}  // namespace aruwsrc
