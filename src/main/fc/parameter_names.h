/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#define PARAM_NAME_GYRO_HARDWARE_LPF "gyro_hardware_lpf"
#define PARAM_NAME_GYRO_LPF1_TYPE "gyro_lpf1_type"
#define PARAM_NAME_GYRO_LPF1_STATIC_HZ "gyro_lpf1_static_hz"
#define PARAM_NAME_GYRO_LPF2_TYPE "gyro_lpf2_type"
#define PARAM_NAME_GYRO_LPF2_STATIC_HZ "gyro_lpf2_static_hz"
#define PARAM_NAME_GYRO_TO_USE "gyro_to_use"
#define PARAM_NAME_ACC_HARDWARE "acc_hardware"
#define PARAM_NAME_ACC_LPF_HZ "acc_lpf_hz"
#define PARAM_NAME_MAG_HARDWARE "mag_hardware"
#define PARAM_NAME_BARO_HARDWARE "baro_hardware"
#define PARAM_NAME_SERIAL_RX_PROVIDER "serialrx_provider"
#define PARAM_NAME_DSHOT_BIDIR "dshot_bidir"
#define PARAM_NAME_USE_UNSYNCED_PWM "use_unsynced_pwm"
#define PARAM_NAME_MOTOR_PWM_PROTOCOL "motor_pwm_protocol"
#define PARAM_NAME_MOTOR_PWM_RATE "motor_pwm_rate"
#define PARAM_NAME_MOTOR_POLES "motor_poles"
#define PARAM_NAME_RATES_TYPE "rates_type"
#define PARAM_NAME_GYRO_CAL_ON_FIRST_ARM "gyro_cal_on_first_arm"
#define PARAM_NAME_DEADBAND "deadband"
#define PARAM_NAME_YAW_DEADBAND "yaw_deadband"
#define PARAM_NAME_PID_PROCESS_DENOM "pid_process_denom"
#define PARAM_NAME_DEBUG_MODE "debug_mode"
#define PARAM_NAME_POSITION_ALTITUDE_SOURCE "altitude_source"
#define PARAM_NAME_POSITION_ALTITUDE_PREFER_BARO "altitude_prefer_baro"
#define PARAM_NAME_POSITION_ALTITUDE_LPF "altitude_lpf"
#define PARAM_NAME_POSITION_ALTITUDE_D_LPF "altitude_d_lpf"

#ifdef USE_GPS
#define PARAM_NAME_GPS_PROVIDER "gps_provider"
#define PARAM_NAME_GPS_SBAS_MODE "gps_sbas_mode"
#define PARAM_NAME_GPS_SBAS_INTEGRITY "gps_sbas_integrity"
#define PARAM_NAME_GPS_AUTO_CONFIG "gps_auto_config"
#define PARAM_NAME_GPS_AUTO_BAUD "gps_auto_baud"
#define PARAM_NAME_GPS_UBLOX_USE_GALILEO "gps_ublox_use_galileo"
#define PARAM_NAME_GPS_UBLOX_ACQUIRE_MODEL "gps_ublox_acquire_model"
#define PARAM_NAME_GPS_UBLOX_FLIGHT_MODEL "gps_ublox_flight_model"
#define PARAM_NAME_GPS_UBLOX_UTC_STANDARD "gps_ublox_utc_standard"
#define PARAM_NAME_GPS_SET_HOME_POINT_ONCE "gps_set_home_point_once"
#define PARAM_NAME_GPS_USE_3D_SPEED "gps_use_3d_speed"
#define PARAM_NAME_GPS_NMEA_CUSTOM_COMMANDS "gps_nmea_custom_commands"
#define PARAM_NAME_GPS_UPDATE_RATE_HZ "gps_update_rate_hz"

#ifdef USE_GPS_RESCUE
#define PARAM_NAME_GPS_RESCUE_MIN_START_DIST "gps_rescue_min_start_dist"
#define PARAM_NAME_GPS_RESCUE_ALT_MODE "gps_rescue_alt_mode"
#define PARAM_NAME_GPS_RESCUE_INITIAL_CLIMB "gps_rescue_initial_climb"
#define PARAM_NAME_GPS_RESCUE_ASCEND_RATE "gps_rescue_ascend_rate"

#define PARAM_NAME_GPS_RESCUE_RETURN_ALT "gps_rescue_return_alt"
#define PARAM_NAME_GPS_RESCUE_GROUND_SPEED "gps_rescue_ground_speed"
#define PARAM_NAME_GPS_RESCUE_MAX_RESCUE_ANGLE "gps_rescue_max_angle"
#define PARAM_NAME_GPS_RESCUE_ROLL_MIX "gps_rescue_roll_mix"
#define PARAM_NAME_GPS_RESCUE_PITCH_CUTOFF "gps_rescue_pitch_cutoff"
#define PARAM_NAME_GPS_RESCUE_IMU_YAW_GAIN "gps_rescue_imu_yaw_gain"

#define PARAM_NAME_GPS_RESCUE_DESCENT_DIST "gps_rescue_descent_dist"
#define PARAM_NAME_GPS_RESCUE_DESCEND_RATE "gps_rescue_descend_rate"
#define PARAM_NAME_GPS_RESCUE_LANDING_ALT "gps_rescue_landing_alt"
#define PARAM_NAME_GPS_RESCUE_DISARM_THRESHOLD "gps_rescue_disarm_threshold"

#define PARAM_NAME_GPS_RESCUE_THROTTLE_MIN "gps_rescue_throttle_min"
#define PARAM_NAME_GPS_RESCUE_THROTTLE_MAX "gps_rescue_throttle_max"
#define PARAM_NAME_GPS_RESCUE_THROTTLE_HOVER "gps_rescue_throttle_hover"

#define PARAM_NAME_GPS_RESCUE_SANITY_CHECKS "gps_rescue_sanity_checks"
#define PARAM_NAME_GPS_RESCUE_MIN_SATS "gps_rescue_min_sats"
#define PARAM_NAME_GPS_RESCUE_ALLOW_ARMING_WITHOUT_FIX "gps_rescue_allow_arming_without_fix"

#define PARAM_NAME_GPS_RESCUE_THROTTLE_P "gps_rescue_throttle_p"
#define PARAM_NAME_GPS_RESCUE_THROTTLE_I "gps_rescue_throttle_i"
#define PARAM_NAME_GPS_RESCUE_THROTTLE_D "gps_rescue_throttle_d"
#define PARAM_NAME_GPS_RESCUE_VELOCITY_P "gps_rescue_velocity_p"
#define PARAM_NAME_GPS_RESCUE_VELOCITY_I "gps_rescue_velocity_i"
#define PARAM_NAME_GPS_RESCUE_VELOCITY_D "gps_rescue_velocity_d"
#define PARAM_NAME_GPS_RESCUE_YAW_P "gps_rescue_yaw_p"

#ifdef USE_MAG
#define PARAM_NAME_GPS_RESCUE_USE_MAG "gps_rescue_use_mag"
#endif
#endif
#endif

#define PARAM_NAME_IMU_DCM_KP "imu_dcm_kp"
#define PARAM_NAME_IMU_DCM_KI "imu_dcm_ki"
#define PARAM_NAME_IMU_PROCESS_DENOM "imu_process_denom"
#ifdef USE_MAG
#define PARAM_NAME_IMU_MAG_DECLINATION "mag_declination"
#endif
