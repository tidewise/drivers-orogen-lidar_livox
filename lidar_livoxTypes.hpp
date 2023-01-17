#ifndef lidar_livox_TYPES_HPP
#define lidar_livox_TYPES_HPP

#include "livox_lidar_def.h"
#include <stdint.h>
#include <string>

/* If you need to define types specific to your oroGen components, define them
 * here. Required headers must be included explicitly
 *
 * However, it is common that you will only import types from your library, in
 * which case you do not need this file
 */

namespace lidar_livox {

    enum LidarModel {
        HAP_T1,
        HAP_TX ,
        MID_360
    };

    struct LidarNetInfo {
        uint16_t cmd_data_port = 5600;
        uint16_t push_msg_port = 0;
        uint16_t point_data_port = 5700;
        uint16_t imu_data_port = 5800;
        uint16_t log_data_port = 5900;
    };

    struct HostNetInfo {
        std::string cmd_data_ip = "192.168.1.50";
        uint16_t cmd_data_port = 5600;
        std::string push_msg_ip = "";
        uint16_t push_msg_port = 0;
        std::string point_data_ip = "192.168.1.50";
        uint16_t point_data_port = 5700;
        std::string imu_data_ip = "192.168.1.50";
        uint16_t imu_data_port = 5800;
        std::string log_data_ip = "";
        uint16_t log_data_port = 5900;
    };

    enum PointDataType {
        IMU_DATA = kLivoxLidarImuData,
        CARTESIAN_COORDINATE_HIGH_DATA = kLivoxLidarCartesianCoordinateHighData,
        CARTESIAN_COORDINATE_LOW_DATA = kLivoxLidarCartesianCoordinateLowData,
        SPHERICAL_COORDINATE_DATA = kLivoxLidarSphericalCoordinateData
    };

    enum ScanPattern {
        NONE_REPETIVE = kLivoxLidarScanPatternNoneRepetive,
        REPETIVE = kLivoxLidarScanPatternRepetive,
        LOW_FRAME_RATE = kLivoxLidarScanPatternRepetiveLowFrameRate
    };

    struct LidarInstallAttitude {
        float roll_deg = 0.0;
        float pitch_deg = 0.0;
        float yaw_deg = 0.0;
        int32_t x = 0; // mm
        int32_t y = 0; // mm
        int32_t z = 0; // mm
    };

    enum LidarWorkMode {
        WORK_MODE_NORMAL = kLivoxLidarNormal,
        WORK_MODE_WAKE_UP = kLivoxLidarWakeUp,
        WORK_MODE_SLEEP = kLivoxLidarSleep,
        WORK_MODE_ERROR = kLivoxLidarError,
        WORK_MODE_POWER_ON_SELF_TEST = kLivoxLidarPowerOnSelfTest,
        WORK_MODE_MOTOR_STARTING = kLivoxLidarMotorStarting,
        WORK_MODE_MOTOR_STOPING = kLivoxLidarMotorStoping,
        WORK_MODE_UPGRADE = kLivoxLidarUpgrade
    };

    struct LivoxLidarLoggerCfgInfo {
        bool lidar_log_enable;
        uint32_t lidar_log_cache_size;
        char lidar_log_path[1024];
    };

    struct LidarIpInfo {
        char ip_addr[16] = {};  /**< IP address. */
        char net_mask[16] = {}; /**< Subnet mask. */
        char gw_addr[16] = {};  /**< Gateway address. */
    };

    struct HostStateInfoIpInfo {
        char host_ip_addr[16] = {}; /**< IP address. */
        uint16_t host_state_info_port = 0;
        uint16_t lidar_state_info_port = 0;
    };

    struct HostPointIPInfo {
        char host_ip_addr[16] = {}; /**< IP address. */
        uint16_t host_point_data_port = 0;
        uint16_t lidar_point_data_port = 0;
    };

    struct HostImuDataIPInfo {
        char host_ip_addr[16] = {};        /**< IP address. */
        uint16_t host_imu_data_port = 0;  // resv
        uint16_t lidar_imu_data_port = 0; // resv
    };

    struct LidarStateInfo {
        PointDataType pcl_data_type;
        ScanPattern pattern_mode;
        lidar_livox::LidarIpInfo livox_lidar_ip_info;
        lidar_livox::HostStateInfoIpInfo host_state_info;
        lidar_livox::HostPointIPInfo host_point_ip_info;
        lidar_livox::HostImuDataIPInfo host_imu_data_ip_info;
        lidar_livox::LidarInstallAttitude install_attitude;

        LidarWorkMode work_mode;
        bool imu_data_en;
        char sn[16] = {};
        char product_info[64] = {};
        uint8_t version_app[4] = {};
        uint8_t version_load[4] = {};
        uint8_t version_hardware[4] = {};
        uint8_t mac[6] = {};
    };
}
#endif
