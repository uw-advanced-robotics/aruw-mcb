#ifndef DJI_MOTOR_TX_HANDLER_HPP_
#define DJI_MOTOR_TX_HANDLER_HPP_

#include <limits.h>

#include "aruwlib/communication/can/can.hpp"
#include "aruwlib/errors/create_errors.hpp"

#include "dji_motor.hpp"

namespace aruwlib
{
namespace motor
{
/**
 * Uses modm can interface to send CAN packets to dji motors. Receiving
 * data is done on the motor level in conjunction with
 * can_receive_interface.hpp.
 *
 * When a motor is instantiated, a CAN interface and a motor number is
 * requested. This class checks when a motor is created and kills
 * the board if multiple identical motors are created.
 *
 * To use this class properly, declare a motor somewhere, preferibly
 * in static memory, and then call the initialize method, which allows
 * one to start interacting with a motor connected via CAN bus. When
 * initialize is called, the motor class is added to a list of motors
 * which a handler runs through, sending and receiving messages for all
 * motors.
 *
 * To receive messages, create a new motor and call the initialize
 * function. This adds the motor to a queue of receive message handlers
 * that is updated by calling the pollCanData function.
 *
 * To send messages, call processCanSendMessage.
 *
 * Currently, no error handling is implemented if you attempt to use
 * a motor without initialization. This must be implemented when
 * error handling is complete.
 */
template <typename Drivers>
class DjiMotorTxHandler
{
public:
    DjiMotorTxHandler() = default;
    DjiMotorTxHandler(const DjiMotorTxHandler<Drivers>&) = delete;
    DjiMotorTxHandler& operator=(const DjiMotorTxHandler<Drivers>&) = delete;

    /**
     * Called when a motor is created, adds to the motor manager
     * and checks to make sure the motor is not already being used.
     */
    void addMotorToManager(DjiMotor<Drivers>* motor)
    {
        // add new motor to either the can1 or can2 motor store
        // because we checked to see if the motor is overloaded, we will
        // never have to worry about overfilling the CanxMotorStore array
        if (motor->getCanBus() == aruwlib::can::CanBus::CAN_BUS1)
        {
            addMotorToManager(can1MotorStore, motor);
        }
        else
        {
            addMotorToManager(can2MotorStore, motor);
        }
    }

    void processCanSendData()
    {
        // set up new can messages to be sent via CAN bus 1 and 2
        modm::can::Message can1MessageLow(CAN_DJI_LOW_IDENTIFIER, CAN_DJI_MESSAGE_SEND_LENGTH);
        can1MessageLow.setExtended(false);

        modm::can::Message can1MessageHigh(CAN_DJI_HIGH_IDENTIFIER, CAN_DJI_MESSAGE_SEND_LENGTH);
        can1MessageHigh.setExtended(false);

        modm::can::Message can2MessageLow(CAN_DJI_LOW_IDENTIFIER, CAN_DJI_MESSAGE_SEND_LENGTH);
        can2MessageLow.setExtended(false);

        modm::can::Message can2MessageHigh(CAN_DJI_HIGH_IDENTIFIER, CAN_DJI_MESSAGE_SEND_LENGTH);
        can2MessageHigh.setExtended(false);

        zeroTxMessage(&can1MessageLow);
        zeroTxMessage(&can1MessageHigh);
        zeroTxMessage(&can2MessageLow);
        zeroTxMessage(&can2MessageHigh);

        serializeMotorStoreSendData(can1MotorStore, &can1MessageLow, &can1MessageHigh);
        serializeMotorStoreSendData(can2MotorStore, &can2MessageLow, &can2MessageHigh);

        if (Drivers::can.isReadyToSend(can::CanBus::CAN_BUS1))
        {
            Drivers::can.sendMessage(can::CanBus::CAN_BUS1, can1MessageLow);
            Drivers::can.sendMessage(can::CanBus::CAN_BUS1, can1MessageHigh);
        }
        if (Drivers::can.isReadyToSend(can::CanBus::CAN_BUS2))
        {
            Drivers::can.sendMessage(can::CanBus::CAN_BUS2, can2MessageLow);
            Drivers::can.sendMessage(can::CanBus::CAN_BUS2, can2MessageHigh);
        }
    }

    void removeFromMotorManager(const DjiMotor<Drivers>& motor)
    {
        if (motor.getCanBus() == aruwlib::can::CanBus::CAN_BUS1)
        {
            removeFromMotorManager(motor, can1MotorStore);
        }
        else
        {
            removeFromMotorManager(motor, can2MotorStore);
        }
    }

    DjiMotor<Drivers> const* getCan1MotorData(MotorId motorId)
    {
        uint32_t normalizedId = DJI_MOTOR_NORMALIZED_ID(motorId);
        if (normalizedId >= DJI_MOTORS_PER_CAN)
        {
            return nullptr;
        }
        return can1MotorStore[normalizedId];
    }

    DjiMotor<Drivers> const* getCan2MotorData(MotorId motorId)
    {
        uint32_t normalizedId = DJI_MOTOR_NORMALIZED_ID(motorId);
        if (normalizedId >= DJI_MOTORS_PER_CAN)
        {
            return nullptr;
        }
        return can2MotorStore[normalizedId];
    }

private:
    static constexpr int DJI_MOTORS_PER_CAN = 8;
    static constexpr int CAN_DJI_MESSAGE_SEND_LENGTH = 8;
    static constexpr uint16_t CAN_DJI_LOW_IDENTIFIER = 0X200;
    static constexpr uint16_t CAN_DJI_HIGH_IDENTIFIER = 0X1FF;

    DjiMotor<Drivers>* can1MotorStore[DJI_MOTORS_PER_CAN] = {0};
    DjiMotor<Drivers>* can2MotorStore[DJI_MOTORS_PER_CAN] = {0};

    void addMotorToManager(DjiMotor<Drivers>** canMotorStore, DjiMotor<Drivers>* const motor)
    {
        int16_t idIndex = DJI_MOTOR_NORMALIZED_ID(motor->getMotorIdentifier());
        bool motorOverloaded = canMotorStore[idIndex] != nullptr;
        bool motorOutOfBounds = (idIndex < 0) || (idIndex >= DJI_MOTORS_PER_CAN);
        // kill start
        modm_assert(!motorOverloaded && !motorOutOfBounds, "can", "motor init", "overloading", 1);
        canMotorStore[idIndex] = motor;
    }

    void serializeMotorStoreSendData(
        DjiMotor<Drivers>** canMotorStore,
        modm::can::Message* messageLow,
        modm::can::Message* messageHigh)
    {
        for (int i = 0; i < DJI_MOTORS_PER_CAN; i++)
        {
            const DjiMotor<Drivers>* const motor = canMotorStore[i];
            if (motor != nullptr)
            {
                if (motor->getMotorIdentifier() - 0x200 <= 4)
                {
                    motor->serializeCanSendData(messageLow);
                }
                else
                {
                    motor->serializeCanSendData(messageHigh);
                }
            }
        }
    }

    void removeFromMotorManager(const DjiMotor<Drivers>& motor, DjiMotor<Drivers>** motorStore)
    {
        uint32_t id = DJI_MOTOR_NORMALIZED_ID(motor.getMotorIdentifier());
        if (motorStore[id] == nullptr)
        {
            // error, trying to remove something that doesn't exist!
            RAISE_ERROR(  // todo fix
                "trying to remove something that doesn't exist",
                aruwlib::errors::Location::MOTOR_CONTROL,
                aruwlib::errors::ErrorType::NULL_MOTOR_ID);
            return;
        }
        motorStore[id] = nullptr;
    }

    void zeroTxMessage(modm::can::Message* message)
    {
        for (int i = 0; i < message->length; i++)
        {
            message->data[i] = 0;
        }
    }
};

}  // namespace motor

}  // namespace aruwlib

#endif  // DJI_MOTOR_TX_HANDLER_HPP_
