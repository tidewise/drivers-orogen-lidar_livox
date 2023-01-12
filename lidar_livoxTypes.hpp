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
        Normal = kLivoxLidarDetectNormal,
        Sensitive = kLivoxLidarDetectSensitive
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
        int32_t yaw_start;
        int32_t yaw_stop;
        int32_t pitch_start;
        int32_t pitch_stop;
        uint32_t rsvd;
    };

    struct LidarFuncIOCfg {
        uint8_t in0;
        uint8_t int1;
        uint8_t out0;
        uint8_t out1;
    };
}
#endif
