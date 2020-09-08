#ifndef CONTROL_OPERATOR_INTERFACE_MOCK_HPP_
#define CONTROL_OPERATOR_INTERFACE_MOCK_HPP_

#include <aruwlib/control/command.hpp>
#include <aruwlib/control/control_operator_interface.hpp>
#include <aruwlib/control/subsystem.hpp>
#include <gmock/gmock.h>

class ControlOperatorInterfaceMock : public aruwlib::control::ControlOperatorInterface
{
public:
    MOCK_METHOD(float, getChassisXInput, (), (override));
    MOCK_METHOD(float, getChassisYInput, (), (override));
    MOCK_METHOD(float, getChassisRInput, (), (override));
    MOCK_METHOD(float, getTurretYawInput, (), (override));
    MOCK_METHOD(float, getTurretPitchInput, (), (override));
    MOCK_METHOD(float, getSentinelSpeedInput, (), (override));
};  // class ControlOperatorInterfaceMock

#endif  // CONTROL_OPERATOR_INTERFACE_MOCK_HPP_
