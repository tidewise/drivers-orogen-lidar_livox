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
        ImuData = kLivoxLidarImuData,
        CartesianCoordinateHighData = kLivoxLidarCartesianCoordinateHighData,
        CartesianCoordinateLowData = kLivoxLidarCartesianCoordinateLowData,
        SphericalCoordinateData = kLivoxLidarSphericalCoordinateData
    };

    enum ScanPattern {
        NoneRepetive = kLivoxLidarScanPatternNoneRepetive,
        Repetive = kLivoxLidarScanPatternRepetive,
        LowFrameRate = kLivoxLidarScanPatternRepetiveLowFrameRate
    };

    enum LidarDetectMode {
        DetectNormal = kLivoxLidarDetectNormal,
        DetectSensitive = kLivoxLidarDetectSensitive
    };

    struct LidarInstallAttitude {
        float roll_deg = 0.0;
        float pitch_deg = 0.0;
        float yaw_deg = 0.0;
        int32_t x = 0; // mm
        int32_t y = 0; // mm
        int32_t z = 0; // mm
    };

    enum LidarGlassHeat {
        StopPowerOnHeatingOrDiagnosticHeating =
            kLivoxLidarStopPowerOnHeatingOrDiagnosticHeating,
        TurnOnHeating = kLivoxLidarTurnOnHeating,
        DiagnosticHeating = kLivoxLidarDiagnosticHeating,
        StopSelfHeating = kLivoxLidarStopSelfHeating
    };

    struct LidarFovCfg {
        int32_t yaw_start = 0;
        int32_t yaw_stop = 0;
        int32_t pitch_start = 0;
        int32_t pitch_stop = 0;
        uint32_t rsvd = 0;
    };

    struct LidarFuncIOCfg {
        uint8_t in0 = 0;
        uint8_t int1 = 0;
        uint8_t out0 = 0;
        uint8_t out1 = 0;
    };

    enum LidarWorkMode {
        WorkModeNormal = kLivoxLidarNormal,
        WorkModeWakeUp = kLivoxLidarWakeUp,
        WorkModeSleep = kLivoxLidarSleep,
        WorkModeError = kLivoxLidarError,
        WorkModePowerOnSelfTest = kLivoxLidarPowerOnSelfTest,
        WorkModeMotorStarting = kLivoxLidarMotorStarting,
        WorkModeMotorStoping = kLivoxLidarMotorStoping,
        WorkModeUpgrade = kLivoxLidarUpgrade
    };

    struct LidarStateInfo {
        PointDataType pcl_data_type = PointDataType::ImuData;
        ScanPattern pattern_mode = ScanPattern::NoneRepetive;
        bool dual_emit_en = false;
        bool point_send_en = false;
        // LivoxLidarIpInfo livox_lidar_ip_info;
        // HostStateInfoIpInfo host_state_info;
        // HostPointIPInfo host_point_ip_info;
        // HostImuDataIPInfo host_imu_data_ip_info;
        // LivoxLidarInstallAttitude install_attitude;
        uint32_t blind_spot_set = 50;
        LidarGlassHeat glass_heat = LidarGlassHeat::StopPowerOnHeatingOrDiagnosticHeating;
        bool fusa_en = false;
        LidarFovCfg fov_cfg0;
        LidarFuncIOCfg fov_cfg1;

        bool fov_en = false;
        LidarDetectMode detect_mode = LidarDetectMode::DetectNormal;
        uint8_t func_io_cfg[4]{0, 0, 0, 0};
        LidarWorkMode work_mode;
        bool imu_data_en = false;
        uint64_t status_code = 0;
        char sn[16] = {};
        char product_info[64] = {};
        uint8_t version_app[4] = {};
        uint8_t version_load[4] = {};
        uint8_t version_hardware[4] = {};
        uint8_t mac[6] = {};

        LidarWorkMode cur_work_state= LidarWorkMode::WorkModeNormal;
        int32_t core_temp= 0;
        uint32_t power_up_cnt= 0;

        uint64_t local_time_now= 0;
        uint64_t last_sync_time= 0;
        int64_t time_offset= 0;

        uint8_t time_sync_type= 0;

        uint16_t diag_status= 0;
        uint8_t fw_type= 0;
        uint32_t hms_code[8]= {};
    };
}
#endif
