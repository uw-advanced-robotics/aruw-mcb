/*
 * Copyright (c) 2023-2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruw-mcb.
 *
 * aruw-mcb is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aruw-mcb is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aruw-mcb.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef MOTOR_TESTER_DRIVERS_HPP_
#define MOTOR_TESTER_DRIVERS_HPP_

#include <cstddef>

#include "tap/drivers.hpp"

#include "aruwsrc/communication/sensors/imu/fused_imu.hpp"
#include "aruwsrc/communication/rtt/rtt_telemetry.hpp"
#include "aruwsrc/display/oled_display.hpp"

namespace aruwsrc::motor_tester
{
class Drivers : public tap::Drivers
{
    friend class DriversSingleton;

#ifdef ENV_UNIT_TESTS
public:
#endif
    static constexpr size_t FUSED_IMU_COUNT = 1;
    using FusedImuType = communication::sensors::imu::FusedImu<FUSED_IMU_COUNT>;

    Drivers()
        : tap::Drivers(),
          // Previous 2x ISM330 fused setup (kept for quick restore):
          // ism330Primary(),
          // ism330Secondary(
          //     aruwsrc::communication::sensors::imu::ism330::ISM330::ChipSelect::
          //         SECONDARY_DIGITAL_OUT_F),
          fusedImu(
              std::array<tap::communication::sensors::imu::AbstractIMU*, FUSED_IMU_COUNT>{
                  &this->mpu6500},
              std::array<tap::algorithms::transforms::Transform, FUSED_IMU_COUNT>{
                  tap::algorithms::transforms::Transform::identity()},
              std::array<FusedImuType::ImuType, FUSED_IMU_COUNT>{
                  FusedImuType::ImuType::MPU6500}),
          rttTelemetry(this),
          oledDisplay(this, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr)
    {
    }

public:
    // Previous 2x ISM330 fused setup:
    // communication::sensors::imu::ism330::ISM330 ism330Primary;
    // communication::sensors::imu::ism330::ISM330 ism330Secondary;
    FusedImuType fusedImu;
    communication::rtt::RttTelemetry rttTelemetry;
    display::OledDisplay oledDisplay;
};  // class aruwsrc::MotortesterDrivers
}  // namespace aruwsrc::motor_tester

#endif  // MOTOR_TESTER_DRIVERS_HPP_
