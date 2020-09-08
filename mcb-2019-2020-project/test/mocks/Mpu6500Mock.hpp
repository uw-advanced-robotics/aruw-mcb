#ifndef MPU6500_MOCK_HPP_
#define MPU6500_MOCK_HPP_

#include <aruwlib/communication/sensors/mpu6500/mpu6500.hpp>
#include <gmock/gmock.h>

class Mpu6500Mock : public aruwlib::sensors::Mpu6500
{
public:
    MOCK_METHOD(void, init, (), (override));
    MOCK_METHOD(void, read, (), (override));
    MOCK_METHOD(bool, initialized, (), (const override));
    MOCK_METHOD(float, getAx, (), (const override));
    MOCK_METHOD(float, getAy, (), (const override));
    MOCK_METHOD(float, getAz, (), (const override));
    MOCK_METHOD(float, getGx, (), (const override));
    MOCK_METHOD(float, getGy, (), (const override));
    MOCK_METHOD(float, getGz, (), (const override));
    MOCK_METHOD(float, getTemp, (), (const override));
    MOCK_METHOD(float, getYaw, (), (override));
    MOCK_METHOD(float, getPitch, (), (override));
    MOCK_METHOD(float, getRoll, (), (override));
    MOCK_METHOD(float, getTiltAngle, (), (override));
};  // Mpu6500Mock

#endif  //  MPU6500_MOCK_HPP_
