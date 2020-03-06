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
#include "src/aruwsrc/control/turret/turret_world_relative_position_command.hpp"
#include "src/aruwsrc/control/chassis/chassis_autorotate_command.hpp"

/* error handling includes --------------------------------------------------*/
#include "src/aruwlib/errors/error_controller.hpp"
#include "src/aruwlib/errors/create_errors.hpp"

using namespace aruwsrc::agitator;
using namespace aruwsrc::control;
using namespace aruwlib::sensors;
using namespace aruwlib;
using namespace aruwsrc::chassis;
using namespace aruwsrc::control;
using namespace aruwlib::sensors;

int i = 0;

/* define subsystems --------------------------------------------------------*/
#if defined(TARGET_SOLDIER)
TurretSubsystem turretSubsystem;
TurretCVCommand turretCVCommand(&turretSubsystem);

ChassisSubsystem soldierChassis;

AgitatorSubsystem agitator17mm(
    AgitatorSubsystem::PID_17MM_P,
    AgitatorSubsystem::PID_17MM_I,
    AgitatorSubsystem::PID_17MM_D,
    AgitatorSubsystem::PID_17MM_MAX_ERR_SUM,
    AgitatorSubsystem::PID_17MM_MAX_OUT,
    AgitatorSubsystem::AGITATOR_GEAR_RATIO_M2006,
    AgitatorSubsystem::AGITATOR_MOTOR_ID,
    AgitatorSubsystem::AGITATOR_MOTOR_CAN_BUS,
    AgitatorSubsystem::isAgitatorInverted
);

ExampleSubsystem frictionWheelSubsystem;

#elif defined(TARGET_SENTRY)
AgitatorSubsystem sentryAgitator(
    AgitatorSubsystem::PID_17MM_P,
    AgitatorSubsystem::PID_17MM_I,
    AgitatorSubsystem::PID_17MM_D,
    AgitatorSubsystem::PID_17MM_MAX_ERR_SUM,
    AgitatorSubsystem::PID_17MM_MAX_OUT,
    AgitatorSubsystem::AGITATOR_GEAR_RATIO_M2006,
    AgitatorSubsystem::AGITATOR_MOTOR_ID,
    AgitatorSubsystem::AGITATOR_MOTOR_CAN_BUS,
    false
);

AgitatorSubsystem sentryKicker(
    AgitatorSubsystem::PID_17MM_KICKER_P,
    AgitatorSubsystem::PID_17MM_KICKER_I,
    AgitatorSubsystem::PID_17MM_KICKER_D,
    AgitatorSubsystem::PID_17MM_KICKER_MAX_ERR_SUM,
    AgitatorSubsystem::PID_17MM_KICKER_MAX_OUT,
    AgitatorSubsystem::AGITATOR_GEAR_RATIO_M2006,
    AgitatorSubsystem::SENTRY_KICKER_MOTOR_ID,
    AgitatorSubsystem::AGITATOR_MOTOR_CAN_BUS,
    false
);

ExampleSubsystem frictionWheelSubsystem;
#endif

/* define commands ----------------------------------------------------------*/

#if defined(TARGET_SOLDIER)
ChassisAutorotateCommand chassisAutorotateCommand(&soldierChassis, &turretSubsystem);
aruwsrc::control::ExampleCommand spinFrictionWheelCommand(&frictionWheelSubsystem,
        ExampleCommand::DEFAULT_WHEEL_RPM);

TurretWorldRelativePositionCommand turretUserCommand(&turretSubsystem, &soldierChassis);

ShootFastComprisedCommand agitatorShootSlowCommand(&agitator17mm);
AgitatorCalibrateCommand agitatorCalibrateCommand(&agitator17mm);
#elif defined(TARGET_SENTRY)
aruwsrc::control::ExampleCommand spinFrictionWheelCommand(&frictionWheelSubsystem,
        ExampleCommand::DEFAULT_WHEEL_RPM);

ShootFastComprisedCommand agitatorShootSlowCommand(&sentryAgitator);
AgitatorCalibrateCommand agitatorCalibrateCommand(&sentryAgitator);
AgitatorRotateCommand agitatorKickerCommand(&sentryKicker, 3.0f, 1, 0, false);
AgitatorCalibrateCommand agitatorCalibrateKickerCommand(&sentryKicker);
#endif

int main()
{
    Board::initialize();

    // Board::DisplaySpiMaster::connect<
    //     Board::DisplayMiso::Miso,
    //     Board::DisplayMosi::Mosi,
    //     Board::DisplaySck::Sck
    // >();

    // SPI1 is on ABP2 which is at 90MHz; use prescaler 64 to get ~fastest baud rate below 1mHz max
    // 90MHz/64=~14MHz
    // Board::DisplaySpiMaster::initialize<Board::SystemClock, 703125_Hz>();

    // aruwlib::display::Sh1106<
    //     Board::DisplaySpiMaster,
    //     Board::DisplayCommand,
    //     Board::DisplayReset,
    //     128, 64,
    //     false
    // > display;
    // display.initializeBlocking();
    // display.setCursor(2, 1);
    // display.setFont(modm::font::ScriptoNarrow);
    // display << "ur code is shit" << modm::endl;
    // display.update();

    aruwlib::algorithms::ContiguousFloatTest contiguousFloatTest;
    contiguousFloatTest.testCore();
    contiguousFloatTest.testBadBounds();
    contiguousFloatTest.testDifference();
    contiguousFloatTest.testRotationBounds();
    contiguousFloatTest.testShiftingValue();
    contiguousFloatTest.testWrapping();

    aruwlib::Remote::initialize();
    aruwlib::sensors::Mpu6500::init();

    aruwlib::serial::RefSerial::getRefSerial().initialize();
    aruwlib::serial::XavierSerial::getXavierSerial().initialize();

    /* register subsystems here ---------------------------------------------*/
    #if defined(TARGET_SOLDIER)
    CommandScheduler::getMainScheduler().registerSubsystem(&agitator17mm);
    CommandScheduler::getMainScheduler().registerSubsystem(&frictionWheelSubsystem);
    CommandScheduler::getMainScheduler().registerSubsystem(&soldierChassis);
    CommandScheduler::getMainScheduler().registerSubsystem(&turretSubsystem);
    #elif defined(TARGET_SENTRY)
    CommandScheduler::getMainScheduler().registerSubsystem(&sentryAgitator);
    CommandScheduler::getMainScheduler().registerSubsystem(&sentryKicker);
    CommandScheduler::getMainScheduler().registerSubsystem(&frictionWheelSubsystem);
    #endif

    /* set any default commands to subsystems here --------------------------*/
    #if defined(TARGET_SOLDIER)
    soldierChassis.setDefaultCommand(&chassisAutorotateCommand);
    turretSubsystem.setDefaultCommand(&turretUserCommand);
    frictionWheelSubsystem.setDefaultCommand(&spinFrictionWheelCommand);
    #elif defined(TARGET_SENTRY)
    frictionWheelSubsystem.setDefaultCommand(&spinFrictionWheelCommand);
    #endif

    /* add any starting commands to the scheduler here ----------------------*/
    #if defined(TARGET_SOLDIER)
    CommandScheduler::getMainScheduler().addCommand(&agitatorCalibrateCommand);
    #elif defined(TARGET_SENTRY)
    CommandScheduler::getMainScheduler().addCommand(&agitatorCalibrateCommand);
    CommandScheduler::getMainScheduler().addCommand(&agitatorCalibrateKickerCommand);
    #endif

    /* register io mappings here --------------------------------------------*/
    #if defined(TARGET_SOLDIER)
    IoMapper::addHoldRepeatMapping(
        IoMapper::newKeyMap(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP),
        &agitatorShootSlowCommand
    );
    #elif defined(TARGET_SENTRY)
    IoMapper::addHoldRepeatMapping(
        IoMapper::newKeyMap(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP),
        &agitatorShootSlowCommand
    );
    IoMapper::addHoldRepeatMapping(
        IoMapper::newKeyMap(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP),
        &agitatorKickerCommand
    );
    #endif

    /* define timers here ---------------------------------------------------*/
    modm::ShortPeriodicTimer updateImuPeriod(2);
    modm::ShortPeriodicTimer sendMotorTimeout(2);

    RAISE_ERROR("fme", aruwlib::errors::Location::TURRET, aruwlib::errors::ErrorType::MOTOR_OFFLINE);
    RAISE_ERROR("fme2", aruwlib::errors::Location::TURRET, aruwlib::errors::ErrorType::MOTOR_ID_OUT_OF_BOUNDS);

    while (1)
    {
        i = __LINE__;
        // // do this as fast as you can
        aruwlib::can::CanRxHandler::pollCanData();
        i = __LINE__;
        aruwlib::serial::XavierSerial::getXavierSerial().updateSerial();
        i = __LINE__;
        aruwlib::serial::RefSerial::getRefSerial().updateSerial();

        i = __LINE__;
        i = __LINE__;
        i = __LINE__;
        Remote::read();
        i = __LINE__;

        i = __LINE__;
        if (updateImuPeriod.execute())
        {
        i = __LINE__;
            Mpu6500::read();
        i = __LINE__;
        }
        i = __LINE__;

        i = __LINE__;
        if (sendMotorTimeout.execute())
        {
            RAISE_ERROR("failed", aruwlib::errors::Location::MPU6500,
                aruwlib::errors::ErrorType::IMU_DATA_NOT_INITIALIZED);
        i = __LINE__;
            RAISE_ERROR("failed", aruwlib::errors::Location::MPU6500,
                aruwlib::errors::ErrorType::IMU_DATA_NOT_INITIALIZED);
        i = __LINE__;
            RAISE_ERROR("failed", aruwlib::errors::Location::MPU6500,
                aruwlib::errors::ErrorType::IMU_DATA_NOT_INITIALIZED);
        i = __LINE__;
            RAISE_ERROR("failed", aruwlib::errors::Location::MPU6500,
                aruwlib::errors::ErrorType::IMU_DATA_NOT_INITIALIZED);
        i = __LINE__;
            RAISE_ERROR("failed", aruwlib::errors::Location::MPU6500,
                aruwlib::errors::ErrorType::IMU_DATA_NOT_INITIALIZED);
        i = __LINE__;
            RAISE_ERROR("failed", aruwlib::errors::Location::MPU6500,
                aruwlib::errors::ErrorType::IMU_DATA_NOT_INITIALIZED);
        i = __LINE__;
            RAISE_ERROR("failed", aruwlib::errors::Location::MPU6500,
                aruwlib::errors::ErrorType::IMU_DATA_NOT_INITIALIZED);
        i = __LINE__;
            RAISE_ERROR("failed", aruwlib::errors::Location::MPU6500,
                aruwlib::errors::ErrorType::IMU_DATA_NOT_INITIALIZED);
        i = __LINE__;
            RAISE_ERROR("failed", aruwlib::errors::Location::MPU6500,
                aruwlib::errors::ErrorType::IMU_DATA_NOT_INITIALIZED);
        i = __LINE__;
            RAISE_ERROR("failed", aruwlib::errors::Location::MPU6500,
                aruwlib::errors::ErrorType::IMU_DATA_NOT_INITIALIZED);
        i = __LINE__;
            RAISE_ERROR("failed", aruwlib::errors::Location::MPU6500,
                aruwlib::errors::ErrorType::IMU_DATA_NOT_INITIALIZED);
        i = __LINE__;
            RAISE_ERROR("failed", aruwlib::errors::Location::MPU6500,
                aruwlib::errors::ErrorType::IMU_DATA_NOT_INITIALIZED);
        i = __LINE__;
            RAISE_ERROR("failed", aruwlib::errors::Location::MPU6500,
                aruwlib::errors::ErrorType::IMU_DATA_NOT_INITIALIZED);
        i = __LINE__;
            RAISE_ERROR("failed", aruwlib::errors::Location::MPU6500,
                aruwlib::errors::ErrorType::IMU_DATA_NOT_INITIALIZED)
        i = __LINE__;
            aruwlib::errors::ErrorController::update();
        i = __LINE__;
            CommandScheduler::getMainScheduler().run();
        i = __LINE__;
            aruwlib::motor::DjiMotorTxHandler::processCanSendData();
        }
        modm::delayMicroseconds(10);
    }
    return 0;
}






void Reset_Handler(void) {
while(1) {}
}						
void NMI_Handler(void) {
while(1) {}
}							
void HardFault_Handler(void) {
while(1) {}
}					
void MemManage_Handler(void) {
while(1) {}
}					
void BusFault_Handler(void) {
while(1) {}
}						
void UsageFault_Handler(void) {
while(1) {}
}					
void SVC_Handler(void) {
while(1) {}
}							
void DebugMon_Handler(void) {
while(1) {}
}						
void PendSV_Handler(void) {
while(1) {}
}						
void SysTick_Handler(void) {
while(1) {}
}						
void WWDG_IRQHandler(void) {
while(1) {}
}						
void PVD_IRQHandler(void) {
while(1) {}
}						
void TAMP_STAMP_IRQHandler(void) {
while(1) {}
}				
void RTC_WKUP_IRQHandler(void) {
while(1) {}
}					
void FLASH_IRQHandler(void) {
while(1) {}
}						
void RCC_IRQHandler(void) {
while(1) {}
}						
void EXTI0_IRQHandler(void) {
while(1) {}
}						
void EXTI1_IRQHandler(void) {
while(1) {}
}						
void EXTI2_IRQHandler(void) {
while(1) {}
}						
void EXTI3_IRQHandler(void) {
while(1) {}
}						
void EXTI4_IRQHandler(void) {
while(1) {}
}						
void DMA1_Stream0_IRQHandler(void) {
while(1) {}
}				
void DMA1_Stream1_IRQHandler(void) {
while(1) {}
}				
void DMA1_Stream2_IRQHandler(void) {
while(1) {}
}				
void DMA1_Stream3_IRQHandler(void) {
while(1) {}
}				
void DMA1_Stream4_IRQHandler(void) {
while(1) {}
}				
void DMA1_Stream5_IRQHandler(void) {
while(1) {}
}				
void DMA1_Stream6_IRQHandler(void) {
while(1) {}
}				
void ADC_IRQHandler(void) {
while(1) {}
}						
void CAN1_TX_IRQHandler(void) {
while(1) {}
}					
void CAN1_RX0_IRQHandler(void) {
while(1) {}
}					
void CAN1_RX1_IRQHandler(void) {
while(1) {}
}					
void CAN1_SCE_IRQHandler(void) {
while(1) {}
}					
void EXTI9_5_IRQHandler(void) {
while(1) {}
}					
void TIM1_BRK_TIM9_IRQHandler(void) {
while(1) {}
}				
void TIM1_UP_TIM10_IRQHandler(void) {
while(1) {}
}				
void TIM1_TRG_COM_TIM11_IRQHandler(void) {
while(1) {}
}		
void TIM1_CC_IRQHandler(void) {
while(1) {}
}					
void TIM2_IRQHandler(void) {
while(1) {}
}						
void TIM3_IRQHandler(void) {
while(1) {}
}						
void TIM4_IRQHandler(void) {
while(1) {}
}						
void I2C1_EV_IRQHandler(void) {
while(1) {}
}					
void I2C1_ER_IRQHandler(void) {
while(1) {}
}					
void I2C2_EV_IRQHandler(void) {
while(1) {}
}					
void I2C2_ER_IRQHandler(void) {
while(1) {}
}					
void SPI1_IRQHandler(void) {
while(1) {}
}						
void SPI2_IRQHandler(void) {
while(1) {}
}						
void USART1_IRQHandler(void) {
while(1) {}
}					
void USART2_IRQHandler(void) {
while(1) {}
}					
void USART3_IRQHandler(void) {
while(1) {}
}					
void EXTI15_10_IRQHandler(void) {
while(1) {}
}					
void RTC_Alarm_IRQHandler(void) {
while(1) {}
}					
void OTG_FS_WKUP_IRQHandler(void) {
while(1) {}
}				
void TIM8_BRK_TIM12_IRQHandler(void) {
while(1) {}
}			
void TIM8_UP_TIM13_IRQHandler(void) {
while(1) {}
}				
void TIM8_TRG_COM_TIM14_IRQHandler(void) {
while(1) {}
}		
void TIM8_CC_IRQHandler(void) {
while(1) {}
}					
void DMA1_Stream7_IRQHandler(void) {
while(1) {}
}				
void FMC_IRQHandler(void) {
while(1) {}
}						
void SDIO_IRQHandler(void) {
while(1) {}
}						
void TIM5_IRQHandler(void) {
while(1) {}
}						
void SPI3_IRQHandler(void) {
while(1) {}
}						
void UART4_IRQHandler(void) {
while(1) {}
}						
void UART5_IRQHandler(void) {
while(1) {}
}						
void TIM6_DAC_IRQHandler(void) {
while(1) {}
}					
void TIM7_IRQHandler(void) {
while(1) {}
}						
void DMA2_Stream0_IRQHandler(void) {
while(1) {}
}				
void DMA2_Stream1_IRQHandler(void) {
while(1) {}
}				
void DMA2_Stream2_IRQHandler(void) {
while(1) {}
}				
void DMA2_Stream3_IRQHandler(void) {
while(1) {}
}				
void DMA2_Stream4_IRQHandler(void) {
while(1) {}
}				
void ETH_IRQHandler(void) {
while(1) {}
}						
void ETH_WKUP_IRQHandler(void) {
while(1) {}
}					
void CAN2_TX_IRQHandler(void) {
while(1) {}
}					
void CAN2_RX0_IRQHandler(void) {
while(1) {}
}					
void CAN2_RX1_IRQHandler(void) {
while(1) {}
}					
void CAN2_SCE_IRQHandler(void) {
while(1) {}
}					
void OTG_FS_IRQHandler(void) {
while(1) {}
}					
void DMA2_Stream5_IRQHandler(void) {
while(1) {}
}				
void DMA2_Stream6_IRQHandler(void) {
while(1) {}
}				
void DMA2_Stream7_IRQHandler(void) {
while(1) {}
}				
void USART6_IRQHandler(void) {
while(1) {}
}					
void I2C3_EV_IRQHandler(void) {
while(1) {}
}					
void I2C3_ER_IRQHandler(void) {
while(1) {}
}					
void OTG_HS_EP1_OUT_IRQHandler(void) {
while(1) {}
}			
void OTG_HS_EP1_IN_IRQHandler(void) {
while(1) {}
}				
void OTG_HS_WKUP_IRQHandler(void) {
while(1) {}
}				
void OTG_HS_IRQHandler(void) {
while(1) {}
}					
void DCMI_IRQHandler(void) {
while(1) {}
}						
void HASH_RNG_IRQHandler(void) {
while(1) {}
}					
void FPU_IRQHandler(void) {
while(1) {}
}						
void UART7_IRQHandler(void) {
while(1) {}
}						
void UART8_IRQHandler(void) {
while(1) {}
}						
void SPI4_IRQHandler(void) {
while(1) {}
}						
void SPI5_IRQHandler(void) {
while(1) {}
}						
void SPI6_IRQHandler(void) {
while(1) {}
}						
void SAI1_IRQHandler(void) {
while(1) {}
}						
void DMA2D_IRQHandler(void) {
while(1) {}
}						