#ifndef DJI_MOTOR_TX_HANDLER_MOCK_HPP_
#define DJI_MOTOR_TX_HANDLER_MOCK_HPP_

#include <aruwlib/motor/dji_motor_tx_handler.hpp>
#include <gmock/gmock.h>

class DjiMotorTxHandlerMock : public aruwlib::motor::DjiMotorTxHandler
{
public:
    MOCK_METHOD(void, addMotorToManager, (aruwlib::motor::DjiMotor * motor), (override));
    MOCK_METHOD(void, processCanSendData, (), (override));
    MOCK_METHOD(void, removeFromMotorManager, (const aruwlib::motor::DjiMotor &motor), (override));
    MOCK_METHOD(
        const aruwlib::motor::DjiMotor *,
        getCan1MotorData,
        (aruwlib::motor::MotorId motorId),
        (override));
    MOCK_METHOD(
        const aruwlib::motor::DjiMotor *,
        getCan2MotorData,
        (aruwlib::motor::MotorId motorId),
        (override));
};  // class DjiMotorTxHandlerMock

#endif  //  DJI_MOTOR_TX_HANDLER_MOCK_HPP_
