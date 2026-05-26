#ifndef DIVERGENCE_WHEEL_SLIP_OBSERVER_HPP_
#define DIVERGENCE_WHEEL_SLIP_OBSERVER_HPP_

#include "tap/algorithms/odometry/chassis_displacement_observer_interface.hpp"
#include "tap/algorithms/odometry/chassis_world_yaw_observer_interface.hpp"
#include "tap/algorithms/odometry/odometry_2d_interface.hpp"
#include "tap/communication/sensors/imu/imu_interface.hpp"
#include "tap/control/chassis/chassis_subsystem_interface.hpp"
#include "tap/drivers.hpp"

#include "aruwsrc/control/chassis/wheel_slip_observer_interface.hpp"

namespace aruwsrc::control::chassis
{
class DivergenceWheelSlipObserver : public WheelSlipObserverInterface,
                                    public tap::control::Subsystem
{
public:
    DivergenceWheelSlipObserver(
        tap::Drivers* drivers,
        const tap::control::chassis::ChassisSubsystemInterface& chassisSubsystem,
        tap::algorithms::odometry::ChassisWorldYawObserverInterface& chassisYawObserver,
        tap::communication::sensors::imu::ImuInterface& imu);

    bool isSlipping() { return slipping; };
    void update();
    void refresh() override { update(); }

private:
    const tap::control::chassis::ChassisSubsystemInterface& chassisSubsystem;
    tap::algorithms::odometry::ChassisWorldYawObserverInterface& chassisYawObserver;
    tap::communication::sensors::imu::ImuInterface& imu;
    uint32_t prevTime = 0;
    float prev_chassis_x_vel = 0;
    float prev_chassis_y_vel = 0;
    float chassisYaw = 0;
    float delta = 0;
    bool slipping = false;
    float divergenceTolerance = 0;
};
}  // namespace aruwsrc::control::chassis
#endif  // DIVERGENCE_WHEEL_SLIP_OBSERVER_HPP_