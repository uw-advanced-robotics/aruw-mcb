#include "motor_homing_command.hpp"

#include "tap/drivers.hpp"

namespace aruwsrc::control
{
    void MotorHomingCommand::initialize() {
        homingState = HomingState::MOVING_TOWARD_LOWER_BOUND;
    }

    void MotorHomingCommand::execute() {
        switch(homingState) {
            case (HomingState::INITIATE_MOVE_TOWARD_LOWER_BOUND) :
            {
                subsystem.setMotorOutput(-HOMING_MOTOR_OUTPUT);
                homingState = HomingState::MOVING_TOWARD_LOWER_BOUND;
                break;
            }
            case (HomingState::MOVING_TOWARD_LOWER_BOUND) :
            {
                if(subsystem.isStalled())
                {
                    subsystem.setLowerBound();
                    subsystem.setMotorOutput(0);
                    homingState = HomingState::MOVING_TOWARD_UPPER_BOUND;
                }
                break;
            }
            case (HomingState::INITIATE_MOVE_TOWARD_UPPER_BOUND) :
            {
                subsystem.setMotorOutput(HOMING_MOTOR_OUTPUT);
                homingState = HomingState::MOVING_TOWARD_UPPER_BOUND;
                break;
            }
            case (HomingState::MOVING_TOWARD_UPPER_BOUND) :
            {
                if(subsystem.isStalled()) {
                    subsystem.setUpperBound();
                    homingState = HomingState::HOMING_COMPLETE;
                }
                break;
            }
            case (HomingState::HOMING_COMPLETE) :
            {
                subsystem.setMotorOutput(0);
                break;
            }
        }

    }

    void MotorHomingCommand::end(bool)
    {
        subsystem.setMotorOutput(0);
    }
    
    bool MotorHomingCommand::isFinished() const {
        return (homingState == HomingState::HOMING_COMPLETE);
    }
    
}