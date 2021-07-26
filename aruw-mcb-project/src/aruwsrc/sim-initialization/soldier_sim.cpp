/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#if defined(TARGET_SOLDIER) && defined(PLATFORM_HOSTED)

#include "tap/motor/motorsim/sim_handler.hpp"

#include "aruwsrc/control/agitator/agitator_subsystem.hpp"
#include "aruwsrc/control/chassis/chassis_subsystem.hpp"
#include "aruwsrc/control/launcher/friction_wheel_subsystem.hpp"
#include "aruwsrc/control/turret/turret_subsystem.hpp"

#include "robot_sim.hpp"

namespace aruwsrc::sim
{
void initialize_robot_sim()
{
    // Register the motor sims for the Agitator subsystem
    // TODO: Create simulator for correct motor
    tap::motorsim::SimHandler::registerSim(
        tap::motorsim::MotorSim::MotorType::M3508,
        aruwsrc::agitator::AgitatorSubsystem::AGITATOR_MOTOR_CAN_BUS,
        aruwsrc::agitator::AgitatorSubsystem::AGITATOR_MOTOR_ID);

    // Register the motor sims for the Chassis subsystem
    tap::motorsim::MotorSim::MotorType CHASSIS_MOTOR_TYPE =
        tap::motorsim::MotorSim::MotorType::M3508;
    tap::motorsim::SimHandler::registerSim(
        CHASSIS_MOTOR_TYPE,
        aruwsrc::chassis::ChassisSubsystem::CAN_BUS_MOTORS,
        aruwsrc::chassis::ChassisSubsystem::LEFT_FRONT_MOTOR_ID);
    tap::motorsim::SimHandler::registerSim(
        CHASSIS_MOTOR_TYPE,
        aruwsrc::chassis::ChassisSubsystem::CAN_BUS_MOTORS,
        aruwsrc::chassis::ChassisSubsystem::LEFT_BACK_MOTOR_ID);
    tap::motorsim::SimHandler::registerSim(
        CHASSIS_MOTOR_TYPE,
        aruwsrc::chassis::ChassisSubsystem::CAN_BUS_MOTORS,
        aruwsrc::chassis::ChassisSubsystem::RIGHT_FRONT_MOTOR_ID);
    tap::motorsim::SimHandler::registerSim(
        CHASSIS_MOTOR_TYPE,
        aruwsrc::chassis::ChassisSubsystem::CAN_BUS_MOTORS,
        aruwsrc::chassis::ChassisSubsystem::RIGHT_BACK_MOTOR_ID);

    // Register the motor sims for the turret subsystem
    tap::motorsim::SimHandler::registerSim(
        tap::motorsim::MotorSim::MotorType::GM6020,
        aruwsrc::control::turret::TurretSubsystem::CAN_BUS_MOTORS,
        aruwsrc::control::turret::TurretSubsystem::PITCH_MOTOR_ID);
    tap::motorsim::SimHandler::registerSim(
        tap::motorsim::MotorSim::MotorType::GM6020,
        aruwsrc::control::turret::TurretSubsystem::CAN_BUS_MOTORS,
        aruwsrc::control::turret::TurretSubsystem::YAW_MOTOR_ID);

    // Register the motor sims for the Hopper Cover (There aren't any)
    // Register the motor sims for the Friction Wheels
    tap::motorsim::SimHandler::registerSim(
        tap::motorsim::MotorSim::MotorType::M3508,
        aruwsrc::launcher::FrictionWheelSubsystem::CAN_BUS_MOTORS,
        aruwsrc::launcher::FrictionWheelSubsystem::LEFT_MOTOR_ID);
    tap::motorsim::SimHandler::registerSim(
        tap::motorsim::MotorSim::MotorType::M3508,
        aruwsrc::launcher::FrictionWheelSubsystem::CAN_BUS_MOTORS,
        aruwsrc::launcher::FrictionWheelSubsystem::RIGHT_MOTOR_ID);
}
}  // namespace aruwsrc::sim

#endif
