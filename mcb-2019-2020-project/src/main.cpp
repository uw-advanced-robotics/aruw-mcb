#include <rm-dev-board-a/board.hpp>
#include <modm/processing/timer.hpp>

/* communication includes ---------------------------------------------------*/
#include "src/aruwlib/communication/sensors/mpu6500/mpu6500.hpp"
#include "src/aruwlib/motor/dji_motor_tx_handler.hpp"
#include "src/aruwlib/communication/can/can_rx_listener.hpp"
#include "src/aruwlib/communication/remote.hpp"
#include "src/aruwlib/communication/serial/xavier_serial.hpp"
#include "src/aruwlib/communication/serial/ref_serial.hpp"
#include "src/aruwlib/display/sh1106.hpp"

/* aruwlib control includes -------------------------------------------------*/
#include "src/aruwlib/control/command_scheduler.hpp"
#include "src/aruwlib/control/controller_mapper.hpp"

/* math includes ------------------------------------------------------------*/
#include "aruwlib/algorithms/contiguous_float_test.hpp"

/* aruwsrc control includes -------------------------------------------------*/
#include "src/aruwsrc/control/example/example_command.hpp"
#include "src/aruwsrc/control/example/example_comprised_command.hpp"
#include "src/aruwsrc/control/example/example_subsystem.hpp"
#include "src/aruwsrc/control/agitator/agitator_subsystem.hpp"
#include "src/aruwsrc/control/agitator/agitator_calibrate_command.hpp"
#include "src/aruwsrc/control/agitator/agitator_shoot_comprised_command_instances.hpp"
#include "src/aruwsrc/control/chassis/chassis_drive_command.hpp"
#include "src/aruwsrc/control/chassis/chassis_subsystem.hpp"
#include "src/aruwsrc/control/turret/turret_subsystem.hpp"
#include "src/aruwsrc/control/turret/turret_cv_command.hpp"
#include "src/aruwsrc/control/turret/turret_init_command.hpp"
#include "src/aruwsrc/control/turret/turret_manual_command.hpp"
#include "src/aruwsrc/control/sentinel/sentinel_drive_manual_command.hpp"
#include "src/aruwsrc/control/sentinel/sentinel_auto_drive_command.hpp"
#include "src/aruwsrc/control/sentinel/sentinel_drive_subsystem.hpp"
#include "src/aruwsrc/control/turret/turret_world_relative_position_command.hpp"
#include "src/aruwsrc/control/chassis/chassis_autorotate_command.hpp"
#include "src/aruwsrc/control/hopper-cover/hopper_subsystem.hpp"
#include "src/aruwsrc/control/hopper-cover/open_hopper_command.hpp"

/* error handling includes --------------------------------------------------*/
#include "src/aruwlib/errors/error_controller.hpp"
#include "src/aruwlib/errors/create_errors.hpp"

void controlFlywheels();
void initFlywheelControl();

int main()
{
    Board::initialize();

    modm::ShortTimeout sendMotorTimeout(2);

    initFlywheelControl();

    while (1)
    {
        // do this as fast as you can
        aruwlib::can::CanRxHandler::pollCanData();
        
        if (sendMotorTimeout.execute())
        {
            sendMotorTimeout.restart(2);
            controlFlywheels();

            aruwlib::motor::DjiMotorTxHandler::processCanSendData();
        }
        modm::delayMicroseconds(10);
    }
    return 0;
}

aruwlib::motor::DjiMotor stateSpaceTestMotor(aruwlib::motor::MOTOR1, aruwlib::can::CanBus::CAN_BUS1, false, "testing motor");

int GEAR_RATIO = 1;
float TORQUE_CONSTANT = 0.3f;
float RESISTANCE = 0.194f;
/// \todo
float MOMENT_OF_INERTIA = 0.3f;
float ANGULAR_VELOCITY_CONSTANT = 24.48f;

// state space constants (1 x 1 matrices)

// system matrix (1x1)
float A_1 = -(GEAR_RATIO * GEAR_RATIO * TORQUE_CONSTANT)
        / (ANGULAR_VELOCITY_CONSTANT * RESISTANCE * MOMENT_OF_INERTIA);

// input matrix (1x1)
float B_1 = (GEAR_RATIO * TORQUE_CONSTANT)
        / (RESISTANCE * MOMENT_OF_INERTIA);

// controller gain matrix (1x1)
/// \todo tune this value
float K_1 = 0.7f;

#include <modm/math/matrix.hpp>

using namespace modm;

Matrix1f A;
Matrix<float, 1, 1> B;
Matrix<float, 1, 1> C;
Matrix<float, 1, 1> D;
Matrix<float, 1, 1> K;
Matrix<float, 1, 1> r;
Matrix<float, 1, 1> x;
Matrix<float, 1, 1> x_hat;
Matrix<float, 1, 1> y;

float desiredRpm = 0.0f;

float x_hatf = 0.0f;
float xf = 0.0f;
float yf = 0.0f;
float uf = 0.0f;

void controlFlywheels2() {
    uf = desiredRpm;
    x_hatf = A_1 * xf + B_1 * uf;
    yf = 1 * xf;
    xf += x_hatf * 0.002f;
    // stateSpaceTestMotor.setDesiredOutput(yf);
}

void initFlywheelControl() {
    A[0][0] = A_1;
    B[0][0] = B_1;
    C[0][0] = 1.0f;
    // D = 0
    K[0][0] = K_1;
}

void controlFlywheels() {
    initFlywheelControl();
    r[0][0] = desiredRpm - stateSpaceTestMotor.getShaftRPM();
    x = (A - B * K) * x + B * K * r;
    y = x;
    stateSpaceTestMotor.setDesiredOutput(y[0][0]);

    // controlFlywheels2();
}

Matrix2f A_aug;
Matrix<float, 2, 1> B_aug;
Matrix<float, 1, 2> C_aug;
Matrix<float, 1, 1> D_aug;
Matrix<float, 1, 2> K_aug;
Matrix<float, 2, 1> r_aug;
Matrix<float, 2, 1> x_aug;
Matrix<float, 1, 1> y_aug;
Matrix<float, 1, 1> u_aug;

void initFlywheelControl_aug() {
    A_aug[0][0] = A_1;
    A_aug[0][1] = B_1;
    B_aug[0][0] = B_1;
    C_aug[0][0] = 1;
    K_aug[0][0] = K_1;
    K_aug[0][1] = 1;
}

void controlFlywheels_aug() {
    initFlywheelControl_aug();
    r_aug[0][0] = desiredRpm - stateSpaceTestMotor.getShaftRPM();
    r_aug[0][1] = (stateSpaceTestMotor.getOutputDesired()
            - stateSpaceTestMotor.getCurrentActual())
            * RESISTANCE; // u_error ? v = ir
    // or should it be as follows:
    r_aug[0][0] = desiredRpm - x_aug[0][0];
    r_aug[0][1] = stateSpaceTestMotor.getOutputDesired() - x_aug[0][1];

    u_aug = K_aug * r_aug;

    x_aug = A_aug * x_aug + B_aug * u_aug;
    y = x;
    stateSpaceTestMotor.setDesiredOutput(y[0][0]);
}