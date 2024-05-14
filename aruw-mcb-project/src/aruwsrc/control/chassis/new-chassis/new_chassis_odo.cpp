// /*
//  * Copyright (c) 2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
//  *
//  * This file is part of aruw-mcb.
//  *
//  * aruw-mcb is free software: you can redistribute it and/or modify
//  * it under the terms of the GNU General Public License as published by
//  * the Free Software Foundation, either version 3 of the License, or
//  * (at your option) any later version.
//  *
//  * aruw-mcb is distributed in the hope that it will be useful,
//  * but WITHOUT ANY WARRANTY; without even the implied warranty of
//  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  * GNU General Public License for more details.
//  *
//  * You should have received a copy of the GNU General Public License
//  * along with aruw-mcb.  If not, see <https://www.gnu.org/licenses/>.
//  */

// #include "new_chassis_odo.hpp"
// #include "aruwsrc/communication/serial/vision_coprocessor.hpp"

// constexpr uint8_t numSwerve = 2;
// constexpr uint8_t numNotSwerve = 2;
// constexpr uint8_t totalMeasurements = 2 * numSwerve + numNotSwerve;

// namespace aruwsrc
// {
// namespace chassis
// {

// ChassisOdometry::ChassisOdometry (
//         std::vector<Wheel*>& wheels,
//         tap::algorithms::odometry::ChassisWorldYawObserverInterface& chassisYawObserver,
//         tap::communication::sensors::imu::ImuInterface& imu,
//         const modm::Vector2f initPos,
//         int numSwerve,
//         int numOther,
//         float cMat[],
//         float qMat[],
//         float rMat[])
//     : wheels(wheels),
//       chassisYawObserver(chassisYawObserver),
//       imu(imu),
//       initPos(initPos),
//       kf(KF_A, cMat, qMat, rMat, KF_P0),
//       numSwerve(numSwerve),
//       numOther(numOther)
//     {

    

//         reset();
//     }

// ChassisOdometry ChassisOdometry::constructChassisOdometry(
//         std::vector<Wheel*>& wheels,
//         tap::algorithms::odometry::ChassisWorldYawObserverInterface& chassisYawObserver,
//         tap::communication::sensors::imu::ImuInterface& imu,
//         const modm::Vector2f initPos,
//         int numSwerve,
//         int numOther)
// {
//             std::vector<float> CMat;
//         std::vector<float> QMat;
//         std::vector<float> P0Mat;
//         std::vector<float> RMat;
//         int totalSize = numSwerve * 2 + numOther;
//         for (const auto& wheel : wheels) {
//             CMat.insert(CMat.end(), wheel->getHMat().begin(), wheel->getHMat().end());
//         }
//         for (int i = 0; i < totalSize * totalSize; ++i) {
//             int row = i / totalSize;
//             int col = i % totalSize;
//             if (row == col) { 
//                 RMat[i] = wheels[i]->config.rConfidence;
//             } else {
//                 RMat[i] = 0.0;
//             }
//         }
//         for (int i = 0; i < totalSize * totalSize; ++i) {
//             int row = i / totalSize;
//             int col = i % totalSize;
//             if (row == col) { 
//                 QMat[i] = wheels[i]->config.initalQValue;
//             } else {
//                 QMat[i] = 0.0;
//             }
//         }

//         float cMatArray[CMat.size()];
//         float qMatArray[QMat.size()];
//         float rMatArray[RMat.size()];
//         for (int i = 0; i < CMat.size(); ++i) {
//             cMatArray[i] = CMat[i];
//         }
//         for (int i = 0; i < QMat.size(); ++i) {
//             qMatArray[i] = QMat[i];
//         }
//         for (int i = 0; i < RMat.size(); ++i) {
//             rMatArray[i] = RMat[i];
//         }
//     return ChassisOdometry(wheels, chassisYawObserver, imu, initPos, numSwerve, numOther, cMatArray, qMatArray, rMatArray);
// }


// void ChassisOdometry::reset()
// {
//     float initialX[int(OdomState::NUM_STATES)] = {initPos.x, 0.0f, initPos.y, 0.0f, 0.0f};
//     kf.init(initialX);
// }

// void ChassisOdometry::update()
// {
//     if (!chassisYawObserver.getChassisWorldYaw(&chassisYaw))
//     {
//         chassisYaw = 0;
//         return;
//     }
//     float z[(numSwerve * 2) + numOther] = {};

//     for (int i = 0; i < ((numSwerve * 2) + numOther); ++i) {
//         for (auto entry : wheels[i]->getHMat()){
//             z[i] = entry;
//             if (wheels[i]->getHMat().size() > 1) {
//                 i++;
//             }
//         }
//     }

//     // perform the update, after this update a new state matrix is now available
//     kf.performUpdate(z);

// }

// void ChassisOdometry::updateChassisStateFromKF(float chassisYaw)
// {
//     const auto& x = kf.getStateVectorAsMatrix();

//     location.setOrientation(chassisYaw);
//     location.setPosition(x[int(OdomState::POS_X)], x[int(OdomState::POS_Y)]);
// }


// }
// }  // namespace aruwsrc::algorithms::odometry
