/*
 * Copyright (c) 2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "sentry_kf_odometry_2d_subsystem.hpp"
#include "sentry_chassis_world_yaw_observer.hpp"

#include "tap/drivers.hpp"
#include "tap/algorithms/math_user_utils.hpp"

using namespace tap::algorithms::transforms;

namespace aruwsrc::sentry
{
SentryKFOdometry2DSubsystem::SentryKFOdometry2DSubsystem(
    tap::Drivers &drivers,
    VisionCoprocessor &visionCoprocessor,
    SentryTransforms &sentryTransforms,
    tap::control::chassis::ChassisSubsystemInterface &chassis,
    SentryChassisWorldYawObserver &yawObserver,
    tap::communication::sensors::imu::ImuInterface &imu,
    modm::Location2D<float> imuToChassisCenter)
    : Subsystem(&drivers),
      visionCoprocessor(visionCoprocessor),
      sentryTransforms(sentryTransforms),
      ChassisKFOdometry(chassis, yawObserver, imu, imuToChassisCenter)
{
}

void SentryKFOdometry2DSubsystem::refresh() {
  VisionCoprocessor::LocalizationCartesianData girlbossMessage = visionCoprocessor.getLastGirlbossLocalizationData();
  VisionCoprocessor::LocalizationCartesianData malewifeMessage = visionCoprocessor.getLastMalewifeLocalizationData();

  // NOTE: The message also contains z, roll, and pitch. But the controls side
  // does not track these values since it is only 2d.  Future work may be to change this.
  float xEstimate;
  float yEstimate;
  float yawEstimate;
  float numEstimates;

  if (!girlbossMessage.isHandled) {
    // Create world to chassis transform
    Transform<CameraFrame, TurretMajorFrame> cameraToTurretMajor = 
      compose<CameraFrame, TurretMinorGirlbossFrame, TurretMajorFrame>(
        sentryTransforms.getTurretGirlbossToCamera().getInverse(),
        sentryTransforms.getTurretMajorToTurretGirlboss().getInverse());
    Transform<CameraFrame, ChassisFrame> cameraToChassis = 
      compose<CameraFrame, TurretMajorFrame, ChassisFrame>(
        cameraToTurretMajor,
        sentryTransforms.getChassisToTurretMajor().getInverse());
    Transform<WorldFrame, CameraFrame> worldToCamera = Transform<CameraFrame, WorldFrame>(
      girlbossMessage.x, girlbossMessage.y, girlbossMessage.z,
      girlbossMessage.roll, girlbossMessage.pitch, girlbossMessage.yaw).getInverse();
    Transform<WorldFrame, ChassisFrame> worldToChassis =
      compose<WorldFrame, CameraFrame, ChassisFrame>(worldToCamera, cameraToChassis);

    // Fill values in estimates
    xEstimate += worldToChassis.getX();
    yEstimate += worldToChassis.getY();
    yawEstimate += worldToChassis.getYaw();
    numEstimates++;
  }
  if (!malewifeMessage.isHandled) {
    Transform<CameraFrame, TurretMajorFrame> cameraToTurretMajor =
      compose<CameraFrame, TurretMinorMalewifeFrame, TurretMajorFrame>(
        sentryTransforms.getTurretMalewifeToCamera().getInverse(),
        sentryTransforms.getTurretMajorToTurretMalewife().getInverse());
    Transform<CameraFrame, ChassisFrame> cameraToChassis = 
      compose<CameraFrame, TurretMajorFrame, ChassisFrame>(
        cameraToTurretMajor,
        sentryTransforms.getChassisToTurretMajor().getInverse());
    Transform<WorldFrame, CameraFrame> worldToCamera = Transform<CameraFrame, WorldFrame>(
      malewifeMessage.x, malewifeMessage.y, malewifeMessage.z,
      malewifeMessage.roll, malewifeMessage.pitch, malewifeMessage.yaw).getInverse();
    Transform<WorldFrame, ChassisFrame> worldToChassis =
      compose<WorldFrame, CameraFrame, ChassisFrame>(worldToCamera, cameraToChassis);

    // Fill values in estimates
    xEstimate += worldToChassis.getX();
    yEstimate += worldToChassis.getY();
    yawEstimate += worldToChassis.getYaw();
    numEstimates++;
  }

  if (numEstimates > 0) {
    // Average between the girlboss and malewife estimates
    xEstimate /= numEstimates;
    yEstimate /= numEstimates;
    yawEstimate /= numEstimates;

    // Rotate velocity and acceleration vectors to get estimates
    float currentYaw;
    chassisYawObserver.getChassisWorldYaw(&currentYaw);
    float yawError = currentYaw - yawEstimate;
    const auto& currentState = kf.getStateVectorAsMatrix();
    float velXEstimate = currentState[int(OdomState::VEL_X)];
    float velYEstimate = currentState[int(OdomState::VEL_Y)];
    float accXEstimate = currentState[int(OdomState::ACC_X)];
    float accYEstimate = currentState[int(OdomState::ACC_Y)];
    tap::algorithms::rotateVector(&velXEstimate, &velYEstimate, yawError);
    tap::algorithms::rotateVector(&accXEstimate, &accYEstimate, yawError);

    // TODO: You still need to store the offset!

    // Replace x and y position, velocity, and acceleration in the Kalman Filter
    float newState[] = { xEstimate, velXEstimate, accXEstimate, yEstimate, velYEstimate, accYEstimate };
    kf.init(newState);
  }

  // Update Kalman Filter
  update();  
}

}  // namespace aruwsrc::sentry
