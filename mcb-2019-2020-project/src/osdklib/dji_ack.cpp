/** @file dji_ack.cpp
 *  @version 3.3
 *  @date April 2017
 *
 *  @brief All DJI OSDK ACK parsing
 *  @brief ACK error API getError and getErrorCodeMessage
 *  to decode received ACK(s).
 *
 *  @Copyright (c) 2017 DJI
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

#include "dji_ack.hpp"
#include "dji_log.hpp"
#include "dji_control.hpp"
#include <string.h>

const bool DJI::OSDK::ACK::SUCCESS = 0;
const bool DJI::OSDK::ACK::FAIL    = 1;

namespace DJI
{
namespace OSDK
{

const std::pair<const uint32_t, const char*> commonData[] = {
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_NONE, (const char*)"MOTOR_FAIL_NONE\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_COMPASS_ABNORMAL, (const char*)"MOTOR_FAIL_COMPASS_ABNORMAL\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_ASSISTANT_PROTECTED, (const char*)"MOTOR_FAIL_ASSISTANT_PROTECTED\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_DEVICE_LOCKED, (const char*)"MOTOR_FAIL_DEVICE_LOCKED\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_IMU_NEED_ADV_CALIBRATION, (const char*)"MOTOR_FAIL_IMU_NEED_ADV_CALIBRATION\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_IMU_SN_ERROR, (const char*)"MOTOR_FAIL_IMU_SN_ERROR\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_IMU_PREHEATING, (const char*)"MOTOR_FAIL_IMU_PREHEATING\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_COMPASS_CALIBRATING, (const char*)"MOTOR_FAIL_COMPASS_CALIBRATING\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_IMU_NO_ATTITUDE, (const char*)"MOTOR_FAIL_IMU_NO_ATTITUDE\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_NO_GPS_IN_NOVICE_MODE, (const char*)"MOTOR_FAIL_NO_GPS_IN_NOVICE_MODE\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_BATTERY_CELL_ERROR, (const char*)"MOTOR_FAIL_BATTERY_CELL_ERROR\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_BATTERY_COMMUNICATION_ERROR, (const char*)"MOTOR_FAIL_BATTERY_COMMUNICATION_ERROR\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_BATTERY_VOLTAGE_TOO_LOW, (const char*)"MOTOR_FAIL_BATTERY_VOLTAGE_TOO_LOW\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_BATTERY_USER_LOW_LAND, (const char*)"MOTOR_FAIL_BATTERY_USER_LOW_LAND\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_BATTERY_MAIN_VOL_LOW, (const char*)"MOTOR_FAIL_BATTERY_MAIN_VOL_LOW\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_BATTERY_TEMP_VOL_LOW, (const char*)"MOTOR_FAIL_BATTERY_TEMP_VOL_LOW\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_BATTERY_SMART_LOW_LAND, (const char*)"MOTOR_FAIL_BATTERY_SMART_LOW_LAND\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_BATTERY_NOT_READY, (const char*)"MOTOR_FAIL_BATTERY_NOT_READY\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_RUNNING_SIMULATOR, (const char*)"MOTOR_FAIL_RUNNING_SIMULATOR\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_PACK_MODE, (const char*)"MOTOR_FAIL_PACK_MODE\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_IMU_ATTI_LIMIT, (const char*)"MOTOR_FAIL_IMU_ATTI_LIMIT\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_NOT_ACTIVATED, (const char*)"MOTOR_FAIL_NOT_ACTIVATED\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_IN_FLYLIMIT_AREA, (const char*)"MOTOR_FAIL_IN_FLYLIMIT_AREA\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_IMU_BIAS_LIMIT, (const char*)"MOTOR_FAIL_IMU_BIAS_LIMIT\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_ESC_ERROR, (const char*)"MOTOR_FAIL_ESC_ERROR\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_IMU_INITING, (const char*)"MOTOR_FAIL_IMU_INITING\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_UPGRADING, (const char*)"MOTOR_FAIL_UPGRADING\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_HAVE_RUN_SIM, (const char*)"MOTOR_FAIL_HAVE_RUN_SIM\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_IMU_CALIBRATING, (const char*)"MOTOR_FAIL_IMU_CALIBRATING\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_TAKEOFF_TILT_TOO_LARGE, (const char*)"MOTOR_FAIL_TAKEOFF_TILT_TOO_LARGE\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_RESERVED_31, (const char*)"MOTOR_FAIL_RESERVED_31\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_RESERVED_32, (const char*)"MOTOR_FAIL_RESERVED_32\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_RESERVED_33, (const char*)"MOTOR_FAIL_RESERVED_33\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_RESERVED_34, (const char*)"MOTOR_FAIL_RESERVED_34\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_RESERVED_35, (const char*)"MOTOR_FAIL_RESERVED_35\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_RESERVED_36, (const char*)"MOTOR_FAIL_RESERVED_36\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_RESERVED_37, (const char*)"MOTOR_FAIL_RESERVED_37\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_RESERVED_38, (const char*)"MOTOR_FAIL_RESERVED_38\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_RESERVED_39, (const char*)"MOTOR_FAIL_RESERVED_39\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_RESERVED_40, (const char*)"MOTOR_FAIL_RESERVED_40\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_INVALID_SN, (const char*)"MOTOR_FAIL_INVALID_SN\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_RESERVED_42, (const char*)"MOTOR_FAIL_RESERVED_42\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_RESERVED_43, (const char*)"MOTOR_FAIL_RESERVED_43\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_FLASH_OPERATING, (const char*)"MOTOR_FAIL_FLASH_OPERATING\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_GPS_DISCONNECT, (const char*)"MOTOR_FAIL_GPS_DISCONNECT\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_INTERNAL_46, (const char*)"MOTOR_FAIL_INTERNAL_46\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_RECORDER_ERROR, (const char*)"MOTOR_FAIL_RECORDER_ERROR\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_INVALID_PRODUCT, (const char*)"MOTOR_FAIL_INVALID_PRODUCT\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_RESERVED_49, (const char*)"MOTOR_FAIL_RESERVED_49\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_RESERVED_50, (const char*)"MOTOR_FAIL_RESERVED_50\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_RESERVED_51, (const char*)"MOTOR_FAIL_RESERVED_51\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_RESERVED_52, (const char*)"MOTOR_FAIL_RESERVED_52\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_RESERVED_53, (const char*)"MOTOR_FAIL_RESERVED_53\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_RESERVED_54, (const char*)"MOTOR_FAIL_RESERVED_54\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_RESERVED_55, (const char*)"MOTOR_FAIL_RESERVED_55\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_RESERVED_56, (const char*)"MOTOR_FAIL_RESERVED_56\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_RESERVED_57, (const char*)"MOTOR_FAIL_RESERVED_57\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_RESERVED_58, (const char*)"MOTOR_FAIL_RESERVED_58\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_RESERVED_59, (const char*)"MOTOR_FAIL_RESERVED_59\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_RESERVED_60, (const char*)"MOTOR_FAIL_RESERVED_60\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_IMU_DISCONNECTED, (const char*)"MOTOR_FAIL_IMU_DISCONNECTED\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_RC_CALIBRATING, (const char*)"MOTOR_FAIL_RC_CALIBRATING\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_RC_CALI_DATA_OUT_RANGE, (const char*)"MOTOR_FAIL_RC_CALI_DATA_OUT_RANGE\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_RC_QUIT_CALI, (const char*)"MOTOR_FAIL_RC_QUIT_CALI\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_RC_CENTER_OUT_RANGE, (const char*)"MOTOR_FAIL_RC_CENTER_OUT_RANGE\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_RC_MAP_ERROR, (const char*)"MOTOR_FAIL_RC_MAP_ERROR\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_WRONG_AIRCRAFT_TYPE, (const char*)"MOTOR_FAIL_WRONG_AIRCRAFT_TYPE\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_SOME_MODULE_NOT_CONFIGURED, (const char*)"MOTOR_FAIL_SOME_MODULE_NOT_CONFIGURED\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_RESERVED_69, (const char*)"MOTOR_FAIL_RESERVED_69\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_RESERVED_70, (const char*)"MOTOR_FAIL_RESERVED_70\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_RESERVED_71, (const char*)"MOTOR_FAIL_RESERVED_71\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_RESERVED_72, (const char*)"MOTOR_FAIL_RESERVED_72\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_RESERVED_73, (const char*)"MOTOR_FAIL_RESERVED_73\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_NS_ABNORMAL, (const char*)"MOTOR_FAIL_NS_ABNORMAL\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_TOPOLOGY_ABNORMAL, (const char*)"MOTOR_FAIL_TOPOLOGY_ABNORMAL\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_RC_NEED_CALI, (const char*)"MOTOR_FAIL_RC_NEED_CALI\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_INVALID_FLOAT, (const char*)"MOTOR_FAIL_INVALID_FLOAT\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_M600_BAT_TOO_FEW, (const char*)"MOTOR_FAIL_M600_BAT_TOO_FEW\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_M600_BAT_AUTH_ERR, (const char*)"MOTOR_FAIL_M600_BAT_AUTH_ERR\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_M600_BAT_COMM_ERR, (const char*)"MOTOR_FAIL_M600_BAT_COMM_ERR\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_M600_BAT_DIF_VOLT_LARGE_1, (const char*)"MOTOR_FAIL_M600_BAT_DIF_VOLT_LARGE_1\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_BATTERY_BOLTAHGE_DIFF_82, (const char*)"MOTOR_FAIL_BATTERY_BOLTAHGE_DIFF_82\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_INVALID_VERSION, (const char*)"MOTOR_FAIL_INVALID_VERSION\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_GIMBAL_GYRO_ABNORMAL, (const char*)"MOTOR_FAIL_GIMBAL_GYRO_ABNORMAL\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_GIMBAL_ESC_PITCH_NO_DATA, (const char*)"MOTOR_FAIL_GIMBAL_ESC_PITCH_NO_DATA\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_GIMBAL_ESC_ROLL_NO_DATA, (const char*)"MOTOR_FAIL_GIMBAL_ESC_ROLL_NO_DATA\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_GIMBAL_ESC_YAW_NO_DATA, (const char*)"MOTOR_FAIL_GIMBAL_ESC_YAW_NO_DATA\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_GIMBAL_FIRM_IS_UPDATING, (const char*)"MOTOR_FAIL_GIMBAL_FIRM_IS_UPDATING\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_GIMBAL_OUT_OF_CONTROL, (const char*)"MOTOR_FAIL_GIMBAL_OUT_OF_CONTROL\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_GIMBAL_PITCH_SHOCK, (const char*)"MOTOR_FAIL_GIMBAL_PITCH_SHOCK\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_GIMBAL_ROLL_SHOCK, (const char*)"MOTOR_FAIL_GIMBAL_ROLL_SHOCK\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_GIMBAL_YAW_SHOCK, (const char*)"MOTOR_FAIL_GIMBAL_YAW_SHOCK\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_IMU_CALI_SUCCESS, (const char*)"MOTOR_FAIL_IMU_CALI_SUCCESS\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_TAKEOFF_EXCEPTION, (const char*)"MOTOR_FAIL_TAKEOFF_EXCEPTION\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_ESC_STALL_NEAR_GOUND, (const char*)"MOTOR_FAIL_ESC_STALL_NEAR_GOUND\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_ESC_UNBALANCE_ON_GRD, (const char*)"MOTOR_FAIL_ESC_UNBALANCE_ON_GRD\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_ESC_PART_EMPTY_ON_GRD, (const char*)"MOTOR_FAIL_ESC_PART_EMPTY_ON_GRD\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_ENGINE_START_FAILED, (const char*)"MOTOR_FAIL_ENGINE_START_FAILED\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_AUTO_TAKEOFF_LAUNCH_FAILED, (const char*)"MOTOR_FAIL_AUTO_TAKEOFF_LAUNCH_FAILED\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_ROLL_OVER_ON_GRD, (const char*)"MOTOR_FAIL_ROLL_OVER_ON_GRD\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_BAT_VERSION_ERR, (const char*)"MOTOR_FAIL_BAT_VERSION_ERR\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_RTK_INITING, (const char*)"MOTOR_FAIL_RTK_INITING\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_RTK_FAIL_TO_INIT, (const char*)"MOTOR_FAIL_RTK_FAIL_TO_INIT\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_RESERVED_104, (const char*)"MOTOR_FAIL_RESERVED_104\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_RESERVED_105, (const char*)"MOTOR_FAIL_RESERVED_105\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_RESERVED_106, (const char*)"MOTOR_FAIL_RESERVED_106\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_RESERVED_107, (const char*)"MOTOR_FAIL_RESERVED_107\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_RESERVED_108, (const char*)"MOTOR_FAIL_RESERVED_108\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_RESERVED_109, (const char*)"MOTOR_FAIL_RESERVED_109\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::START_MOTOR_FAIL_MOTOR_STARTED, (const char*)"START_MOTOR_FAIL_MOTOR_STARTED\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_INTERNAL_111, (const char*)"MOTOR_FAIL_INTERNAL_111\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_ESC_CALIBRATING, (const char*)"MOTOR_FAIL_ESC_CALIBRATING\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_GPS_SIGNATURE_INVALID, (const char*)"MOTOR_FAIL_GPS_SIGNATURE_INVALID\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_GIMBAL_CALIBRATING, (const char*)"MOTOR_FAIL_GIMBAL_CALIBRATING\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_FORCE_DISABLE, (const char*)"MOTOR_FAIL_FORCE_DISABLE\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::TAKEOFF_HEIGHT_EXCEPTION, (const char*)"TAKEOFF_HEIGHT_EXCEPTION\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_ESC_NEED_UPGRADE, (const char*)"MOTOR_FAIL_ESC_NEED_UPGRADE\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_GYRO_DATA_NOT_MATCH, (const char*)"MOTOR_FAIL_GYRO_DATA_NOT_MATCH\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_APP_NOT_ALLOW, (const char*)"MOTOR_FAIL_APP_NOT_ALLOW\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_COMPASS_IMU_MISALIGN, (const char*)"MOTOR_FAIL_COMPASS_IMU_MISALIGN\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_FLASH_UNLOCK, (const char*)"MOTOR_FAIL_FLASH_UNLOCK\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_ESC_SCREAMING, (const char*)"MOTOR_FAIL_ESC_SCREAMING\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_ESC_TEMP_HIGH, (const char*)"MOTOR_FAIL_ESC_TEMP_HIGH\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_BAT_ERR, (const char*)"MOTOR_FAIL_BAT_ERR\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::IMPACT_IS_DETECTED, (const char*)"IMPACT_IS_DETECTED\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_MODE_FAILURE, (const char*)"MOTOR_FAIL_MODE_FAILURE\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_CRAFT_FAIL_LATELY, (const char*)"MOTOR_FAIL_CRAFT_FAIL_LATELY\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::KILL_SWITCH_ON, (const char*)"KILL_SWITCH_ON\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::CommonACK::MOTOR_FAIL_MOTOR_CODE_ERROR, (const char*)"MOTOR_FAIL_MOTOR_CODE_ERROR\n")
};

const std::map<const uint32_t, const char*>
ACK::createCommonErrorCodeMap()
{
  const std::map<const uint32_t, const char*> errorCodeMap(
    commonData, commonData + sizeof commonData / sizeof commonData[0]);
  return errorCodeMap;
}

const std::pair<const uint32_t, const char*> activateData[] = {
  std::make_pair(OpenProtocolCMD::DJIErrorCode::ActivationACK::SUCCESS,
                 (const char*)"ACTIVATE_SUCCESS\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::ActivationACK::ACCESS_LEVEL_ERROR,
                 (const char*)"ACCESS_LEVEL_ERROR\n"),
  std::make_pair(
    OpenProtocolCMD::DJIErrorCode::ActivationACK::DJIGO_APP_NOT_CONNECTED,
    (const char*)"DJIGO_APP_NOT_CONNECTED_ERROR\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::ActivationACK::ENCODE_ERROR,
                 (const char*)"ENCODE_ERROR\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::ActivationACK::NETWORK_ERROR,
                 (const char*)"NETWORK_ERROR\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::ActivationACK::NEW_DEVICE_ERROR,
                 (const char*)"NEW_DEVICE_ERROR\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::ActivationACK::OSDK_VERSION_ERROR,
                 (const char*)"OSDK_VERSION_ERROR\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::ActivationACK::PARAMETER_ERROR,
                 (const char*)"PARAMETER_ERROR\n"),
  std::make_pair(
    OpenProtocolCMD::DJIErrorCode::ActivationACK::SERVER_ACCESS_REFUSED,
    (const char*)"SERVER_ACCESS_REFUSED\n")
};

const std::map<const uint32_t, const char*>
ACK::createActivateErrorCodeMap()
{
  const std::map<const uint32_t, const char*> errorCodeMap(
    activateData, activateData + sizeof activateData / sizeof activateData[0]);
  return errorCodeMap;
}

const std::pair<const uint32_t, const char*> subscribeData[] = {
  std::make_pair(OpenProtocolCMD::DJIErrorCode::SubscribeACK::SUCCESS,
                 (const char*)"SUBSCRIBER_SUCCESS\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::SubscribeACK::ILLEGAL_FREQUENCY,
                 (const char*)"SUBSCRIBER_ILLEGAL_FREQUENCY\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::SubscribeACK::ILLEGAL_DATA_LENGTH,
                 (const char*)"SUBSCRIBER_ILLEGAL_DATA_LENGTH\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::SubscribeACK::ILLEGAL_UID,
                 (const char*)"SUBSCRIBER_ILLEGAL_UID\n"),
  std::make_pair(
    OpenProtocolCMD::DJIErrorCode::SubscribeACK::INCORRECT_NUM_OF_TOPICS,
    (const char*)"SUBSCRIBER_INCORRECT_NUM_OF_TOPICS\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::SubscribeACK::INTERNAL_ERROR_0X09,
                 (const char*)"SUBSCRIBER_INTERNAL_ERROR_0X09\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::SubscribeACK::INTERNAL_ERROR_0X4A,
                 (const char*)"SUBSCRIBER_INTERNAL_ERROR_0X4A\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::SubscribeACK::FAILED_AUTHENTICATION,
                 (const char*)"SUBSCRIBER_FAILED_AUTHENTICATION\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::SubscribeACK::INTERNAL_ERROR_0XFF,
                 (const char*)"SUBSCRIBER_INTERNAL_ERROR_0XFF\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::SubscribeACK::MULTIPLE_SUBSCRIBE,
                 (const char*)"SUBSCRIBER_MULTIPLE_SUBSCRIBE\n"),
  std::make_pair(
    OpenProtocolCMD::DJIErrorCode::SubscribeACK::PACKAGE_ALREADY_EXISTS,
    (const char*)"SUBSCRIBER_PACKAGE_ALREADY_EXISTS\n"),
  std::make_pair(
    OpenProtocolCMD::DJIErrorCode::SubscribeACK::PACKAGE_DOES_NOT_EXIST,
    (const char*)"SUBSCRIBER_PACKAGE_DOES_NOT_EXIST\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::SubscribeACK::PACKAGE_EMPTY,
                 (const char*)"SUBSCRIBER_PACKAGE_EMPTY\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::SubscribeACK::PACKAGE_OUT_OF_RANGE,
                 (const char*)"SUBSCRIBER_PACKAGE_OUT_OF_RANGE\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::SubscribeACK::PACKAGE_TOO_LARGE,
                 (const char*)"SUBSCRIBER_PACKAGE_TOO_LARGE\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::SubscribeACK::PAUSED,
                 (const char*)"SUBSCRIBER_PAUSED\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::SubscribeACK::PERMISSION_DENY,
                 (const char*)"SUBSCRIBER_PERMISSION_DENY\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::SubscribeACK::PIPELINE_OVERFLOW,
                 (const char*)"SUBSCRIBER_PIPELINE_OVERFLOW\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::SubscribeACK::RESUMED,
                 (const char*)"SUBSCRIBER_RESUMED\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::SubscribeACK::SOURCE_DEVICE_OFFLINE,
                 (const char*)"SUBSCRIBER_SOURCE_DEVICE_OFFLINE\n"),
  std::make_pair(
    OpenProtocolCMD::DJIErrorCode::SubscribeACK::VERSION_DOES_NOT_MATCH,
    (const char*)"SUBSCRIBER_VERSION_DOES_NOT_MATCH\n"),
  std::make_pair(
    OpenProtocolCMD::DJIErrorCode::SubscribeACK::VERSION_UNKNOWN_ERROR,
    (const char*)"SUBSCRIBER_VERSION_UNKNOWN_ERROR\n"),
  std::make_pair(
    OpenProtocolCMD::DJIErrorCode::SubscribeACK::VERSION_VERSION_TOO_FAR,
    (const char*)"SUBSCRIBER_VERSION_VERSION_TOO_FAR\n")
};

const std::map<const uint32_t, const char*>
ACK::createSubscribeErrorCodeMap()
{
  const std::map<const uint32_t, const char*> errorCodeMap(
    subscribeData,
    subscribeData + sizeof subscribeData / sizeof subscribeData[0]);
  return errorCodeMap;
}

const std::pair<const uint32_t, const char*> setControlData[] = {
  std::make_pair(OpenProtocolCMD::DJIErrorCode::ControlACK::SetControl::
                   IOC_OBTAIN_CONTROL_ERROR,
                 (const char*)"IOC_OBTAIN_CONTROL_ERROR\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::ControlACK::SetControl::
                   OBTAIN_CONTROL_IN_PROGRESS,
                 (const char*)"OBTAIN_CONTROL_IN_PROGRESS\n"),
  std::make_pair(
    OpenProtocolCMD::DJIErrorCode::ControlACK::SetControl::OBTAIN_CONTROL_SUCCESS,
    (const char*)"OBTAIN_CONTROL_SUCCESS\n"),
  std::make_pair(
    OpenProtocolCMD::DJIErrorCode::ControlACK::SetControl::RC_MODE_ERROR,
    (const char*)"RC_MODE_ERROR\n"),
  std::make_pair(
    OpenProtocolCMD::DJIErrorCode::ControlACK::SetControl::RC_NEED_MODE_F,
    (const char*)"RC_NEED_MODE_F\n"),
  std::make_pair(
    OpenProtocolCMD::DJIErrorCode::ControlACK::SetControl::RC_NEED_MODE_P,
    (const char*)"RC_NEED_MODE_P\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::ControlACK::SetControl::
                   RELEASE_CONTROL_IN_PROGRESS,
                 (const char*)"RELEASE_CONTROL_IN_PROGRESS\n"),
  std::make_pair(
    OpenProtocolCMD::DJIErrorCode::ControlACK::SetControl::RELEASE_CONTROL_SUCCESS,
    (const char*)"RELEASE_CONTROL_SUCCESS\n")
};

const std::map<const uint32_t, const char*>
ACK::createSetControlErrorCodeMap()
{
  const std::map<const uint32_t, const char*> errorCodeMap(
    setControlData,
    setControlData + sizeof setControlData / sizeof setControlData[0]);
  return errorCodeMap;
}

/*
 * SetArm supported only in Matrice 100
 */
const std::pair<const uint32_t, const char*> setArmData[] = {
  std::make_pair(OpenProtocolCMD::DJIErrorCode::ControlACK::SetArm::SUCCESS,
                 (const char*)"SET_ARM_SUCCESS\n"),
  std::make_pair(
    OpenProtocolCMD::DJIErrorCode::ControlACK::SetArm::AIRCRAFT_IN_AIR_ERROR,
    (const char*)"SET_ARM_AIRCRAFT_IN_AIR_ERROR\n"),
  std::make_pair(
    OpenProtocolCMD::DJIErrorCode::ControlACK::SetArm::ALREADY_ARMED_ERROR,
    (const char*)"SET_ARM_ALREADY_ARMED_ERROR\n"),
  std::make_pair(
    OpenProtocolCMD::DJIErrorCode::ControlACK::SetArm::OBTAIN_CONTROL_NEEDED_ERROR,
    (const char*)"SET_ARM_OBTAIN_CONTROL_NEEDED_ERROR\n")
};

const std::map<const uint32_t, const char*>
ACK::createSetArmErrorCodeMap()
{
  const std::map<const uint32_t, const char*> errorCodeMap(
    setArmData, setArmData + sizeof setArmData / sizeof setArmData[0]);
  return errorCodeMap;
}

const std::pair<const uint32_t, const char*> taskData[] = {
  std::make_pair(OpenProtocolCMD::DJIErrorCode::ControlACK::Task::SUCCESS,
                 (const char*)"CONTROLLER_TASK_SUCCESS\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::ControlACK::Task::ALREADY_PACKED,
                 (const char*)"CONTROLLER_ALREADY_PACKED\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::ControlACK::Task::ALREADY_RUNNING,
                 (const char*)"CONTROLLER_ALREADY_RUNNING\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::ControlACK::Task::BAD_GPS,
                 (const char*)"CONTROLLER_BAD_GPS\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::ControlACK::Task::BAD_SENSOR,
                 (const char*)"CONTROLLER_BAD_SENSOR\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::ControlACK::Task::GIMBAL_MOUNTED,
                 (const char*)"CONTROLLER_GIMBAL_MOUNTED\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::ControlACK::Task::INVAILD_COMMAND,
                 (const char*)"CONTROLLER_INVAILD_COMMAND\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::ControlACK::Task::IN_AIR,
                 (const char*)"CONTROLLER_IN_AIR"),
  std::make_pair(
    OpenProtocolCMD::DJIErrorCode::ControlACK::Task::IN_SIMULATOR_MODE,
    (const char*)"CONTROLLER_IN_SIMULATOR_MODE\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::ControlACK::Task::MOTOR_OFF,
                 (const char*)"CONTROLLER_MOTOR_OFF\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::ControlACK::Task::MOTOR_ON,
                 (const char*)"CONTROLLER_MOTOR_ON\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::ControlACK::Task::NOT_IN_AIR,
                 (const char*)"CONTROLLER_NOT_IN_AIR\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::ControlACK::Task::NOT_RUNNING,
                 (const char*)"CONTROLLER_NOT_RUNNING\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::ControlACK::Task::NO_HOMEPOINT,
                 (const char*)"CONTROLLER_NO_HOMEPOINT\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::ControlACK::Task::NO_LANDING_GEAR,
                 (const char*)"CONTROLLER_NO_LANDING_GEAR\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::ControlACK::Task::NO_PACKED,
                 (const char*)"CONTROLLER_NO_PACKED\n"),
  std::make_pair(
    OpenProtocolCMD::DJIErrorCode::ControlACK::Task::PACKED_MODE_NOT_SUPPORTED,
    (const char*)"CONTROLLER_PACKED_MODE_DATATYPE_NOT_SUPPORTED_\n")
};

const std::map<const uint32_t, const char*>
ACK::createTaskErrorCodeMap()
{
  const std::map<const uint32_t, const char*> errorCodeMap(
    taskData, taskData + sizeof taskData / sizeof taskData[0]);
  return errorCodeMap;
}

/*
 * Supported in Matrice 100
 */
const std::pair<const uint32_t, const char*> LegacyTaskData[] = {
  std::make_pair(OpenProtocolCMD::DJIErrorCode::ControlACK::LegacyTask::SUCCESS,
                 (const char*)"CONTROLLER_SUCCESS\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::ControlACK::LegacyTask::FAIL,
                 (const char*)"CONTROLLER_FAIL\n")
};

const std::map<const uint32_t, const char*>
ACK::createLegacyTaskErrorCodeMap()
{
  const std::map<const uint32_t, const char*> errorCodeMap(
    LegacyTaskData, LegacyTaskData + sizeof LegacyTaskData / sizeof LegacyTaskData[0]);
  return errorCodeMap;
}

const std::pair<const uint32_t, const char*> missionData[] = {
  std::make_pair(OpenProtocolCMD::DJIErrorCode::MissionACK::Common::AT_NO_FLY_ZONE,
                 (const char*)"MISSION_AT_NO_FLY_ZONE\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::MissionACK::Common::BAD_GPS,
                 (const char*)"MISSION_BAD_GPS\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::MissionACK::Common::RTK_NOT_READY,
                 (const char*)"MISSION_RTK_NOT_READY\n"),
  std::make_pair(
    OpenProtocolCMD::DJIErrorCode::MissionACK::Common::BEGGINER_MODE_NOT_SUPPORTED,
    (const char*)"MISSION_BEGGINER_MODE_NOT_SUPPORTED\n"),
  std::make_pair(
    OpenProtocolCMD::DJIErrorCode::MissionACK::Common::CLOSE_IOC_REQUIRED,
    (const char*)"MISSION_CLOSE_IOC_REQUIRED\n"),
  std::make_pair(
    OpenProtocolCMD::DJIErrorCode::MissionACK::Common::CONDITIONS_NOT_SATISFIED,
    (const char*)"MISSION_CONDITIONS_NOT_SATISFIED\n"),
  std::make_pair(
    OpenProtocolCMD::DJIErrorCode::MissionACK::Common::CROSSING_NO_FLY_ZONE,
    (const char*)"MISSION_CROSSING_NO_FLY_ZONE\n"),
  std::make_pair(
    OpenProtocolCMD::DJIErrorCode::MissionACK::Common::INVALID_COMMAND,
    (const char*)"MISSION_INVALID_COMMAND\n"),
  std::make_pair(
    OpenProtocolCMD::DJIErrorCode::MissionACK::Common::INVALID_PARAMETER,
    (const char*)"MISSION_INVALID_PARAMETER"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::MissionACK::Common::IN_PROGRESS,
                 (const char*)"MISSION_IN_PROGRESS\n"),
  std::make_pair(
    OpenProtocolCMD::DJIErrorCode::MissionACK::Common::LANDING_IN_PROGRESS,
    (const char*)"MISSION_LANDING_IN_PROGRESS\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::MissionACK::Common::LOW_BATTERY,
                 (const char*)"MISSION_LOW_BATTERY\n"),
  std::make_pair(
    OpenProtocolCMD::DJIErrorCode::MissionACK::Common::NOT_INITIALIZED,
    (const char*)"MISSION_NOT_INITIALIZED\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::MissionACK::Common::NOT_RUNNING,
                 (const char*)"MISSION_NOT_RUNNING\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::MissionACK::Common::NOT_SUPPORTED,
                 (const char*)"MISSION_NOT_SUPPORTED\n"),
  std::make_pair(
    OpenProtocolCMD::DJIErrorCode::MissionACK::Common::OBTAIN_CONTROL_REQUIRED,
    (const char*)"MISSION_OBTAIN_CONTROL_REQUIRED\n"),
  std::make_pair(
    OpenProtocolCMD::DJIErrorCode::MissionACK::Common::OTHER_MISSION_RUNNING,
    (const char*)"OTHER_MISSION_RUNNING\n"),
  std::make_pair(
    OpenProtocolCMD::DJIErrorCode::MissionACK::Common::RC_NOT_IN_MODE_F,
    (const char*)"MISSION_RC_NOT_IN_MODE_F\n"),
  std::make_pair(
    OpenProtocolCMD::DJIErrorCode::MissionACK::Common::RRETURN_HOME_IN_PROGRESS,
    (const char*)"MISSION_RRETURN_HOME_IN_PROGRESS\n"),
  std::make_pair(
    OpenProtocolCMD::DJIErrorCode::MissionACK::Common::START_MOTORS_IN_PROGRESS,
    (const char*)"MISSION_START_MOTORS_IN_PROGRESS\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::MissionACK::Common::SUCCESS,
                 (const char*)"MISSION_SUCCESS\n"),
  std::make_pair(
    OpenProtocolCMD::DJIErrorCode::MissionACK::Common::TAKE_OFF_IN_PROGRESS,
    (const char*)"MISSION_TAKE_OFF_IN_PROGRESS\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::MissionACK::Common::TASK_TIMEOUT,
                 (const char*)"MISSION_TASK_TIMEOUT\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::MissionACK::Common::
                   TOO_FAR_FROM_CURRENT_POSITION,
                 (const char*)"MISSION_TOO_FAR_FROM_CURRENT_POSITION\n"),
  std::make_pair(
    OpenProtocolCMD::DJIErrorCode::MissionACK::Common::TOO_FAR_FROM_HOME,
    (const char*)"MISSION_TOO_FAR_FROM_HOME\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::MissionACK::Common::TOO_HIGH,
                 (const char*)"MISSION_TOO_HIGH\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::MissionACK::Common::TOO_LOW,
                 (const char*)"MISSION_TOO_LOW\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::MissionACK::Common::UNKNOWN_ERROR,
                 (const char*)"MISSION_UNKNOWN_ERROR\n"),
  std::make_pair(
    OpenProtocolCMD::DJIErrorCode::MissionACK::Common::UNRECORDED_HOME,
    (const char*)"MISSION_UNRECORDED_HOME\n"),
  std::make_pair(
    OpenProtocolCMD::DJIErrorCode::MissionACK::Common::VEHICLE_DID_NOT_TAKE_OFF,
    (const char*)"MISSION_VEHICLE_DID_NOT_TAKE_OFF\n"),
  std::make_pair(
    OpenProtocolCMD::DJIErrorCode::MissionACK::Common::WRONG_WAYPOINT_INDEX,
    (const char*)"MISSION_WRONG_WAYPOINT_INDEX\n"),
  std::make_pair(
    OpenProtocolCMD::DJIErrorCode::MissionACK::Follow::CUTOFF_TIME_OVERFLOW,
    (const char*)"FOLLOW_MISSION_CUTOFF_TIME_OVERFLOW\n"),
  std::make_pair(
    OpenProtocolCMD::DJIErrorCode::MissionACK::Follow::GIMBAL_PITCH_ANGLE_OVERFLOW,
    (const char*)"FOLLOW_MISSION_GIMBAL_PITCH_ANGLE_OVERFLOW\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::MissionACK::Follow::
                   TOO_FAR_FROM_YOUR_POSITION_LACK_OF_RADIO_CONNECTION,
                 (const char*)"FOLLOW_MISSION_TOO_FAR_FROM_YOUR_POSITION_LACK_"
                              "OF_RADIO_CONNECTION\n"),
  std::make_pair(
    OpenProtocolCMD::DJIErrorCode::MissionACK::HotPoint::FAILED_TO_PAUSE,
    (const char*)"HOTPOINT_MISSION_FAILED_TO_PAUSE\n"),
  std::make_pair(
    OpenProtocolCMD::DJIErrorCode::MissionACK::HotPoint::INVALID_DIRECTION,
    (const char*)"HOTPOINT_MISSION_INVALID_DIRECTION\n"),
  std::make_pair(
    OpenProtocolCMD::DJIErrorCode::MissionACK::HotPoint::
      INVALID_LATITUDE_OR_LONGITUTE,
    (const char*)"HOTPOINT_MISSION_INVALID_LATITUDE_OR_LONGITUTE\n"),
  std::make_pair(
    OpenProtocolCMD::DJIErrorCode::MissionACK::HotPoint::INVALID_PAREMETER,
    (const char*)"HOTPOINT_MISSION_INVALID_PAREMETER\n"),
  std::make_pair(
    OpenProtocolCMD::DJIErrorCode::MissionACK::HotPoint::INVALID_RADIUS,
    (const char*)"HOTPOINT_MISSION_INVALID_RADIUS\n"),
  std::make_pair(
    OpenProtocolCMD::DJIErrorCode::MissionACK::HotPoint::INVALID_START_POINT,
    (const char*)"HOTPOINT_MISSION_INVALID_VISION\n"),
  std::make_pair(
    OpenProtocolCMD::DJIErrorCode::MissionACK::HotPoint::INVALID_YAW_MODE,
    (const char*)"HOTPOINT_MISSION_INVALID_YAW_MODE\n"),
  std::make_pair(
    OpenProtocolCMD::DJIErrorCode::MissionACK::HotPoint::IN_PAUSED_MODE,
    (const char*)"HOTPOINT_MISSION_IN_PAUSED_MODE\n"),
  std::make_pair(
    OpenProtocolCMD::DJIErrorCode::MissionACK::HotPoint::TOO_FAR_FROM_HOTPOINT,
    (const char*)"HOTPOINT_MISSION_TOO_FAR_FROM_HOTPOINT\n"),
  std::make_pair(
    OpenProtocolCMD::DJIErrorCode::MissionACK::HotPoint::YAW_RATE_OVERFLOW,
    (const char*)"HOTPOINT_MISSION_YAW_RATE_OVERFLOW\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::MissionACK::WayPoint::CHECK_FAILED,
                 (const char*)"WAYPOINT_MISSION_CHECK_FAILED\n"),
  std::make_pair(
    OpenProtocolCMD::DJIErrorCode::MissionACK::WayPoint::DATA_NOT_ENOUGH,
    (const char*)"WAYPOINT_MISSION_DATA_NOT_ENOUGH\n"),
  std::make_pair(
    OpenProtocolCMD::DJIErrorCode::MissionACK::WayPoint::TRACE_TOO_LONG,
    (const char*)"WAYPOINT_MISSION_DISTANCE_OVERFLOW\n"),
  std::make_pair(
    OpenProtocolCMD::DJIErrorCode::MissionACK::WayPoint::INVALID_ACTION,
    (const char*)"WAYPOINT_MISSION_INVALID_ACTION\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::MissionACK::WayPoint::INVALID_DATA,
                 (const char*)"WAYPOINT_MISSION_INVALID_DATA\n"),
  std::make_pair(
    OpenProtocolCMD::DJIErrorCode::MissionACK::WayPoint::INVALID_POINT_DATA,
    (const char*)"WAYPOINT_MISSION_INVALID_POINT_DATA\n"),
  std::make_pair(
    OpenProtocolCMD::DJIErrorCode::MissionACK::WayPoint::INVALID_VELOCITY,
    (const char*)"WAYPOINT_MISSION_INVALID_VELOCITY\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::MissionACK::WayPoint::IN_PROGRESS,
                 (const char*)"MISSION_IN_PROGRESS\n"),
  std::make_pair(
    OpenProtocolCMD::DJIErrorCode::MissionACK::WayPoint::NOT_IN_PROGRESS,
    (const char*)"WAYPOINT_MISSION_NOT_IN_PROGRESS\n"),
  std::make_pair(
    OpenProtocolCMD::DJIErrorCode::MissionACK::WayPoint::POINTS_NOT_ENOUGH,
    (const char*)"WAYPOINT_MISSION_POINTS_NOT_ENOUGH\n"),
  std::make_pair(
    OpenProtocolCMD::DJIErrorCode::MissionACK::WayPoint::POINTS_TOO_CLOSE,
    (const char*)"WAYPOINT_MISSION_POINTS_TOO_CLOSE\n"),
  std::make_pair(
    OpenProtocolCMD::DJIErrorCode::MissionACK::WayPoint::POINTS_TOO_FAR,
    (const char*)"WAYPOINT_MISSION_POINTS_TOO_FAR\n"),
  std::make_pair(
    OpenProtocolCMD::DJIErrorCode::MissionACK::WayPoint::POINT_DATA_NOT_ENOUGH,
    (const char*)"WAYPOINT_MISSION_POINT_DATA_NOT_ENOUGH\n"),
  std::make_pair(
    OpenProtocolCMD::DJIErrorCode::MissionACK::WayPoint::POINT_OVERFLOW,
    (const char*)"WAYPOINT_MISSION_POINT_OVERFLOW\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::MissionACK::WayPoint::TOTAL_DISTANCE_TOO_LONG,
                 (const char*)"WAYPOINT_MISSION_TIMEOUT\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::MissionACK::IOC::TOO_CLOSE_TO_HOME,
                 (const char*)"IOC_MISSION_TOO_CLOSE_TO_HOME\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::MissionACK::IOC::UNKNOWN_TYPE,
                 (const char*)"IOC_MISSION_UNKNOWN_TYPE\n")
};

const std::map<const uint32_t, const char*>
ACK::createMissionErrorCodeMap()
{
  const std::map<const uint32_t, const char*> errorCodeMap(
    missionData, missionData + sizeof missionData / sizeof missionData[0]);
  return errorCodeMap;
}

const std::pair<const uint32_t, const char*> mfioData[] = {
  std::make_pair(OpenProtocolCMD::DJIErrorCode::MFIOACK::init::PORT_DATA_ERROR,
                 (const char*)"MFIO_INIT_PORT_DATA_ERROR\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::MFIOACK::init::PORT_MODE_ERROR,
                 (const char*)"MFIO_INIT_PORT_MODE_ERROR\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::MFIOACK::init::PORT_NUMBER_ERROR,
                 (const char*)"MFIO_INIT_PORT_NUMBER_ERROR\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::MFIOACK::init::SUCCESS,
                 (const char*)"MFIO_INIT_SUCCESS\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::MFIOACK::init::UNKNOWN_ERROR,
                 (const char*)"RC_NEED_MODE_F\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::MFIOACK::set::CHANNEL_ERROR,
                 (const char*)"MFIO_SET_CHANNEL_ERROR\n"),
  std::make_pair(
    OpenProtocolCMD::DJIErrorCode::MFIOACK::set::PORT_NOT_MAPPED_ERROR,
    (const char*)"MFIO_SET_PORT_NOT_MAPPED_ERROR\n"),
  std::make_pair(OpenProtocolCMD::DJIErrorCode::MFIOACK::set::SUCCESS,
                 (const char*)"MFIO_SET_SUCCESS\n")
};

const std::map<const uint32_t, const char*>
ACK::createMFIOErrorCodeMap()
{
  const std::map<const uint32_t, const char*> errorCodeMap(
    mfioData, mfioData + sizeof mfioData / sizeof mfioData[0]);
  return errorCodeMap;
}

bool
ACK::getError(ACK::ErrorCode ack)
{
  const uint8_t cmd[] = { ack.info.cmd_set, ack.info.cmd_id };

  if (ack.info.cmd_set == OpenProtocolCMD::CMDSet::activation)
  {
    return (ack.data == OpenProtocolCMD::DJIErrorCode::ActivationACK::SUCCESS)
             ? ACK::SUCCESS
             : ACK::FAIL;
  }
  else if (ack.info.cmd_set == OpenProtocolCMD::CMDSet::broadcast)
  {
    // Push Data, no ACK
  }
  else if (memcmp(cmd, OpenProtocolCMD::CMDSet::Control::setControl,
                  sizeof(cmd)) == 0)
  {
    if (ack.info.buf[2] == ACK::RELEASE_CONTROL)
    { //! Data is set at buf + SET_CMD_SIZE which is buf + 2;
      // Release control was called
      return (ack.data == OpenProtocolCMD::DJIErrorCode::ControlACK::SetControl::
                            RELEASE_CONTROL_SUCCESS)
               ? ACK::SUCCESS
               : ACK::FAIL;
    }
    else if (ack.info.buf[2] == ACK::OBTAIN_CONTROL)
    {
      // Obtain control was called
      return (ack.data == OpenProtocolCMD::DJIErrorCode::ControlACK::SetControl::
                            OBTAIN_CONTROL_SUCCESS)
               ? ACK::SUCCESS
               : ACK::FAIL;
    }
  }
  else if (memcmp(cmd, OpenProtocolCMD::CMDSet::Control::setArm, sizeof(cmd)) ==
           0)
  {
    /*
     * SetArm command supported in Matrice 100/ M600 old firmware
     */
    return (ack.data == OpenProtocolCMD::DJIErrorCode::ControlACK::SetArm::SUCCESS)
             ? ACK::SUCCESS
             : ACK::FAIL;
  }
  else if (memcmp(cmd, OpenProtocolCMD::CMDSet::Control::control,
                  sizeof(cmd)) == 0)
  {
    // Does not return an ACK
  }
  else if (memcmp(cmd, OpenProtocolCMD::CMDSet::Control::task, sizeof(cmd)) ==
           0)
  {
    if (ack.info.version == Version::FW(3,2,15,62))
    {
      //! ACKs supported in Matrice 600 old firmware
      return (ack.data ==
          OpenProtocolCMD::DJIErrorCode::ControlACK::LegacyTask::SUCCESS)
             ? ACK::SUCCESS
             : ACK::FAIL;
    }
    else if (ack.info.version != Version::M100_31)
    {
      return (ack.data == OpenProtocolCMD::DJIErrorCode::ControlACK::Task::SUCCESS)
               ? ACK::SUCCESS
               : ACK::FAIL;
    }
    else
    {
      //! ACKs supported in Matrice 100
      return (ack.data ==
              OpenProtocolCMD::DJIErrorCode::ControlACK::LegacyTask::SUCCESS)
               ? ACK::SUCCESS
               : ACK::FAIL;
    }
  }
  else if (memcmp(cmd, OpenProtocolCMD::CMDSet::Control::killSwitch, sizeof(cmd)) ==
           0)
  {
    return (ack.data ==
      OpenProtocolCMD::DJIErrorCode::ControlACK::KillSwitch::SUCCESS)
           ? ACK::SUCCESS : ACK::FAIL;
  }
  else if (ack.info.cmd_set == OpenProtocolCMD::CMDSet::mission)
  {
    return (ack.data == OpenProtocolCMD::DJIErrorCode::MissionACK::Common::SUCCESS)
             ? ACK::SUCCESS
             : ACK::FAIL;
  }
  else if (ack.info.cmd_set == OpenProtocolCMD::CMDSet::hardwareSync)
  {
    // Verify ACK
  }
  else if (ack.info.cmd_set == OpenProtocolCMD::CMDSet::virtualRC)
  {
    // Deprecated in 3.2.20

    // TODO implement for backward compatibility
  }
  else if (ack.info.cmd_set == OpenProtocolCMD::CMDSet::subscribe)
  {
    if (memcmp(cmd, OpenProtocolCMD::CMDSet::Subscribe::pauseResume,
               sizeof(cmd)) == 0)
    {
      return (ack.data == OpenProtocolCMD::DJIErrorCode::SubscribeACK::PAUSED ||
              ack.data == OpenProtocolCMD::DJIErrorCode::SubscribeACK::RESUMED)
               ? ACK::SUCCESS
               : ACK::FAIL;
    }
    else
    {
      return (ack.data == OpenProtocolCMD::DJIErrorCode::SubscribeACK::SUCCESS)
               ? ACK::SUCCESS
               : ACK::FAIL;
    }
  }
  else if (ack.info.cmd_set == OpenProtocolCMD::CMDSet::mfio)
  {
    if (memcmp(cmd, OpenProtocolCMD::CMDSet::MFIO::get, sizeof(cmd)) == 0)
    {
      return (ack.data == OpenProtocolCMD::DJIErrorCode::MFIOACK::get::SUCCESS)
               ? ACK::SUCCESS
               : ACK::FAIL;
    }
    else
    {
      return (ack.data == OpenProtocolCMD::DJIErrorCode::MFIOACK::init::SUCCESS)
               ? ACK::SUCCESS
               : ACK::FAIL;
    }
  }

  return ACK::FAIL;
}

/*
 * @note Log Error Message
 */
void
ACK::getErrorCodeMessage(ACK::ErrorCode ack, const char* func)
{
  DSTATUS("%s", func);
  switch (ack.info.cmd_set)
  {
    case OpenProtocolCMD::CMDSet::activation:
      // CMD_ID agnostic
      getCMDSetActivationMSG(ack);
      break;
    case OpenProtocolCMD::CMDSet::control:
      // Get message by CMD_ID
      getCMDSetControlMSG(ack);
      break;
    case OpenProtocolCMD::CMDSet::broadcast:
      getSetBroadcastMSG(ack);
      break;
    case OpenProtocolCMD::CMDSet::mission:
      // CMD_ID agnostic
      getCMDSetMissionMSG(ack);
      break;
    case OpenProtocolCMD::CMDSet::hardwareSync:
      getCMDSetSyncMSG(ack);
      break;
    case OpenProtocolCMD::CMDSet::virtualRC:
      getCMDSetVirtualRCMSG(ack);
      break;
    case OpenProtocolCMD::CMDSet::mfio:
      getCMDSetMFIOMSG(ack);
      break;
    case OpenProtocolCMD::CMDSet::subscribe:
      // CMD_ID agnostic
      getCMDSetSubscribeMSG(ack);
      break;
    default:
      getCommonErrorCodeMessage(ack);
      break;
  }
}

/*
 * @note CMD_ID agnostic
 */
void
ACK::getCMDSetActivationMSG(ACK::ErrorCode ack)
{
  const std::map<const uint32_t, const char*> activateErrorCodeMap =
    ACK::createActivateErrorCodeMap();
  auto msg = activateErrorCodeMap.find(ack.data);

  if (msg != activateErrorCodeMap.end())
  {
    DSTATUS(msg->second);
  }
  else
  {
    getCommonErrorCodeMessage(ack);
  }
}

void
ACK::getCommonErrorCodeMessage(ACK::ErrorCode ack)
{
  const std::map<const uint32_t, const char*> commonErrorCodeMap =
    ACK::createCommonErrorCodeMap();
  auto msg = commonErrorCodeMap.find(ack.data);

  if (msg != commonErrorCodeMap.end())
  {
    DERROR(msg->second);
  }
  else
  {
    DERROR("UNKNOWN_ACK_ERROR_CODE\n");
  }
}

void
ACK::getCMDSetSubscribeMSG(ACK::ErrorCode ack)
{
  const std::map<const uint32_t, const char*> subscribeErrorCodeMap =
    ACK::createSubscribeErrorCodeMap();
  auto msg = subscribeErrorCodeMap.find(ack.data);

  if (msg != subscribeErrorCodeMap.end())
  {
    DSTATUS(msg->second);
  }
  else
  {
    DSTATUS("UNKNOWN_SUBSCRIBER_ACK_ERROR_CODE_0x%X\n", ack.data);
  }
}

void
ACK::getCMDSetControlMSG(ACK::ErrorCode ack)
{
  const uint8_t cmd[] = { ack.info.cmd_set, ack.info.cmd_id };

  if (memcmp(cmd, OpenProtocolCMD::CMDSet::Control::setControl, sizeof(cmd)) ==
      0)
  {
    getCMDIDSetControlMSG(ack.data, ack.info.version);
  }
  else if (memcmp(cmd, OpenProtocolCMD::CMDSet::Control::control,
                  sizeof(cmd)) == 0)
  {
    getCMDIDControlMSG(ack);
  }
  else if (memcmp(cmd, OpenProtocolCMD::CMDSet::Control::task, sizeof(cmd)) ==
           0)
  {
    /*
     * @note Deprecated in 3.2.20
     */
    getCMDIDTaskMSG(ack);
  }
  else if (memcmp(cmd, OpenProtocolCMD::CMDSet::Control::setArm, sizeof(cmd)) ==
           0)
  {
    /*
     * SetArm command supported in Matrice 100
     */
    getCMDIDSetArmMSG(ack);
  }
}

void
ACK::getCMDIDSetControlMSG(uint8_t ack, Version::FirmWare version)
{
  const std::map<const uint32_t, const char*> setControlErrorCodeMap =
    ACK::createSetControlErrorCodeMap();
  auto msg = setControlErrorCodeMap.find(ack);

  if (msg != setControlErrorCodeMap.end())
  {
    if (msg->first ==
        OpenProtocolCMD::DJIErrorCode::ControlACK::SetControl::RC_MODE_ERROR)
    {
      if (version != Version::M100_31 && version != Version::A3_31)
      {
        DSTATUS("RC_NEED_MODE_P\n");
      }
      else
      {
        DSTATUS("RC_NEED_MODE_F\n");
      }
    }
    DSTATUS(msg->second);
  }
  else
  {
    DSTATUS("UNKNOWN_ACK_ERROR_CODE\n");
  }
}

void
ACK::getCMDIDControlMSG(ACK::ErrorCode ack)
{
}

void
ACK::getCMDIDTaskMSG(ACK::ErrorCode ack)
{
  std::map<const uint32_t, const char*> taskErrorCodeMap;

  if (ack.info.version == Version::FW(3,2,15,62))
  {
    taskErrorCodeMap = static_cast<std::map<const uint32_t, const char*>>(ACK::createLegacyTaskErrorCodeMap());
  }
  else if (ack.info.version == Version::M100_31)
  {
    taskErrorCodeMap = static_cast<std::map<const uint32_t, const char*>>(ACK::createLegacyTaskErrorCodeMap());
  }
  else
  {
    taskErrorCodeMap = static_cast<std::map<const uint32_t, const char*>>(ACK::createTaskErrorCodeMap());
  }

  auto msg = taskErrorCodeMap.find(ack.data);

  if (ack.info.buf != nullptr)
  {
    uint8_t taskCmd = *ack.info.buf;
    if ((taskCmd != (uint8_t)Control::FlightCommand::takeOff) && msg != taskErrorCodeMap.end())
    {
      DSTATUS(msg->second);
    }
    else
    {
      getCommonErrorCodeMessage(ack);
    }
  }
  else
  {
    DERROR("ACK INFO BUF IS NULL\n");
  }
}

/*
 * SetArm command supported on Matrice 100 only
 */
void
ACK::getCMDIDSetArmMSG(ACK::ErrorCode ack)
{
  const std::map<const uint32_t, const char*> setArmErrorCodeMap =
    ACK::createSetArmErrorCodeMap();
  auto msg = setArmErrorCodeMap.find(ack.data);

  if (msg != setArmErrorCodeMap.end())
  {
    DSTATUS(msg->second);
  }
  else
  {
    getCommonErrorCodeMessage(ack);
  }
}

void
ACK::getSetBroadcastMSG(ACK::ErrorCode ack)
{
}

/*
 * @note CMD_ID agnostic
 *
 * @todo Check DJI_ERROR_CODE for comments
 */
void
ACK::getCMDSetMissionMSG(ACK::ErrorCode ack)
{
  const std::map<const uint32_t, const char*> missionErrorCodeMap =
    ACK::createMissionErrorCodeMap();
  auto msg = missionErrorCodeMap.find(ack.data);

  if (msg != missionErrorCodeMap.end())
  {
    DSTATUS(msg->second);
  }
  else
  {
    DSTATUS("UNKNOWN_MISSION_ACK_ERROR_CODE\n");
  }
}

void
ACK::getCMDSetSyncMSG(ACK::ErrorCode ack)
{
}

void
ACK::getCMDSetVirtualRCMSG(ACK::ErrorCode ack)
{
}

void
ACK::getCMDSetMFIOMSG(ACK::ErrorCode ack)
{
  const std::map<const uint32_t, const char*> mfioErrorCodeMap =
    ACK::createMFIOErrorCodeMap();
  auto msg = mfioErrorCodeMap.find(ack.data);

  if (msg != mfioErrorCodeMap.end())
  {
    DSTATUS(msg->second);
  }
  else
  {
    DSTATUS("MFIO_UNKNOWN_ERROR\n");
  }
}

} // namespace OSDK
} // namespace DJI
