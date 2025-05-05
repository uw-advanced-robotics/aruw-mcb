/*
 * Copyright (c) 2020-2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

 #ifndef TORQUE_CONTROLLED_MOTOR_HPP_
 #define TORQUE_CONTROLLED_MOTOR_HPP_
 
 #include "tap/motor/motor_interface.hpp"
 
 #include "modm/math/interpolation/linear.hpp"
 
/**
 * @brief Class for controlling a motor using torque control when that motor supports a current control.
 * 
 * Designed to take in a torque value and map it to a motor output using linear interpolation.
 * or, provided the motor is sufficiently linear, a simple mapping using the kt value.
 */

 namespace aruwsrc::control::motor
 {
 class TorqueControlledMotor
 {
 public:
    /**
     * @brief Constructor for TorqueControlledMotor.
     * @param[in] motor Reference to the motor interface.
     * @param[in] kt Torque constant for the motor.
     * @param[in] maxOutput Maximum output value of the motor.
     */
     TorqueControlledMotor(tap::motor::MotorInterface& motor, float kt, int32_t maxOutput);


    /**
     * @brief Constructor for TorqueControlledMotor with custom mapping.
     * @param[in] motor Reference to the motor interface.
     * @param[in] in_out_map Pointer to an array of pairs representing the input-output mapping.
     * @param[in] size Size of the mapping array.
     */
     TorqueControlledMotor(tap::motor::MotorInterface& motor, const modm::Pair<float, float>* in_out_map, uint8_t size);

     void setTorque(const float torque);
 
     inline tap::motor::MotorInterface& getMotor() const;
 
 private:
     tap::motor::MotorInterface& motor;
 
     modm::interpolation::Linear<modm::Pair<float, float>> torqueDesInterpolator;

     modm::Pair<float, float> ktMap[2]; 
 };
 
 }  // namespace aruwsrc::control::motor
 
 #endif  // TORQUE_CONTROLLED_MOTOR_HPP_