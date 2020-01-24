// Subsystem for grabber mechanism

#include "src/aruwsrc/control/grabber_subsystem.hpp"

#include <rm-dev-board-a/board.hpp>

namespace aruwsrc
{

namespace control
{
    void GrabberSubsystem::refresh(void) {
        grabberDigitalOutPin::set(gripMode); 
    }

    void GrabberSubsystem::setMovement(bool gripMode) {
        this->gripMode = gripMode; 
        grabberDigitalOutPin::set(gripMode);
    }
}  // namespace control

}  // namespace aruwsrc
