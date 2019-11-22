#include <rm-dev-board-a/board.hpp>
#include <modm/container/smart_pointer.hpp>

#include "src/control/scheduler.hpp"
#include "src/control/example-command.hpp"
#include "src/control/example-subsystem.hpp"
#include "src/motor/dji_motor_tx_handler.hpp"
#include "src/communication/can/can_rx_listener.hpp"
#include "src/aruwsrc/control/agitator_subsystem.hpp"
#include "src/aruwsrc/control/agitator_rotate_command.hpp"

#include <modm/processing/timer.hpp>

modm::ShortPeriodicTimer t(2);

using namespace aruwsrc::control;

// ExampleSubsystem frictionWheelSubsystem;
AgitatorSubsystem agitator17mm(36);

aruwlib::motor::DjiMotor* m3;

float f = 0.0f;

using namespace std;

int count = 0;

aruwsrc::control::ExampleCommand* commandWatch;

typedef enum {
    WAITING,
    RUNNING,
    BACK_UNJAM,
    FORWARD_UNJAM
} AgitatorUnjamStates;

AgitatorUnjamStates UJ;



int main()
{
    Board::initialize();

    // create new commands
    // modm::SmartPointer frictionWheelDefaultCommand(
    //     new aruwsrc::control::ExampleCommand(&frictionWheelSubsystem));
    modm::SmartPointer rotateAgitator(
        new AgitatorRotateCommand(&agitator17mm, PI / 2.0f)
    );

    // add commands if necessary
    // frictionWheelSubsystem.SetDefaultCommand(frictionWheelDefaultCommand);

    // commandWatch = reinterpret_cast<aruwsrc::control::ExampleCommand*>
    //     (frictionWheelDefaultCommand.getPointer());

    // add commands when necessary
    aruwlib::control::CommandScheduler::addCommand(rotateAgitator);

    // reinterpret_cast<aruwsrc::control::AgitatorRotateCommand*>(frictionWheelDefaultCommand.getPointer())->initialize();
    bool prevRead = false;

    if (1 && !1)  // person tells agitator to rotate
    {
        modm::SmartPointer rotateAgitator(new AgitatorRotateCommand(&agitator17mm, PI / 2.0f));
        aruwlib::control::CommandScheduler::addCommand(rotateAgitator);
    }
    if (agitator17mm.isAgitatorJammed())
    {
        modm::SmartPointer unjamAgitator(new AgitatorRotateCommand(&agitator17mm, - PI / 2.0f));
        aruwlib::control::CommandScheduler::addCommand(unjamAgitator);
    }
    if (1 /*agitator back unjam complete*/)
    {
        // agitator foward unjam
    }
    if (1 /*that is done*/)
    {
        // if that is done without jam, it is unjammed, otherwise, go back to jam backwards state
    }
    // we either make the unjam command cover everything necessary for an unjam or we add some sort
    // of command group/state machine

    while (1)
    {
        switch(UJ)
        {
            case WAITING:
            {
                if (1)
                {
                    modm::SmartPointer rotateAgitator(
                        new AgitatorRotateCommand(&agitator17mm, PI / 2.0f));
                    aruwlib::control::CommandScheduler::addCommand(rotateAgitator);
                }

                break;
            }
            case RUNNING:
            {
                if (agitator17mm.isAgitatorJammed())
                {
                    modm::SmartPointer unjamAgitator(
                        new AgitatorRotateCommand(&agitator17mm, - PI / 2.0f));
                    aruwlib::control::CommandScheduler::addCommand(unjamAgitator);
                }
                break;
            }
            case BACK_UNJAM:
            {
                break;
            }
            case FORWARD_UNJAM:
            {
                break;
            }
        };

        if (Board::Button::read() == true && !prevRead)
        {
            agitator17mm.agitatorCalibrateHere();
        }
        prevRead = Board::Button::read();

        f = agitator17mm.agitatorEncoderToPosition();
        aruwlib::can::CanRxHandler::pollCanData();

        // run scheduler
        count++;
        if (t.execute())
        {
            count = 0;
        //    aruwlib::control::CommandScheduler::run();
            // aruwlib::motor::DjiMotorTxHandler::processCanSendData();
             agitator17mm.refresh();

        }

        // do this as fast as you can
        aruwlib::can::CanRxHandler::pollCanData();

        modm::delayMicroseconds(10);
    }
    return 0;
}
