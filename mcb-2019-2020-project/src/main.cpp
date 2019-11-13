#include <rm-dev-board-a/board.hpp>
#include "src/algorithms/pid.hpp"
#include "src/motor/dji_motor.hpp"
#include "src/motor/dji_motor_tx_handler.hpp"
#include "src/communication/can/can_rx_handler.hpp"

#define MOTOR_KP 10.0f
#define MOTOR_KI 0.0f
#define MOTOR_KD 0.0f
#define MOTOR_MAX_OUTPUT 30000
#define MOTOR_MAX_SUM 30000

int main()
{
    Board::initialize();
    aruwlib::motor::DjiMotor m1(
        aruwlib::motor::MotorId::MOTOR1,
        aruwlib::can::CanBus::CAN_BUS1
    );
    aruwlib::algorithms::Pid<float, 9> motorPid(
        MOTOR_KP,
        MOTOR_KI,
        MOTOR_KD,
        MOTOR_MAX_OUTPUT,
        MOTOR_MAX_SUM
    );
    int16_t desRPM = 0;
    int16_t desOutput = 0;
    uint32_t lastSent = modm::Clock::now().getTime();
    motorPid.setParameter({
        MOTOR_KP,
        MOTOR_KI,
        MOTOR_KD,
        MOTOR_MAX_OUTPUT,
        MOTOR_MAX_SUM});
    while (1)
    {
        aruwlib::can::CanRxHandler::pollCanData();

        uint32_t currTime = modm::Clock::now().getTime();
        Board::Leds::toggle();
        if (currTime - lastSent >= 3) {
            int16_t currRPM = m1.getShaftRPM();
            motorPid.update(desRPM - currRPM);
            desOutput = (int16_t) motorPid.getValue();
            m1.setDesiredOutput(desOutput);
            aruwlib::motor::DjiMotorTxHandler::processCanSendData();
            lastSent = currTime;
        }
        modm::delayNanoseconds(300);
    }
    return 0;
}
