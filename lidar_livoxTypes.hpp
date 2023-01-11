#ifndef lidar_livox_TYPES_HPP
#define lidar_livox_TYPES_HPP

#include <string>
#include <stdint.h>

/* If you need to define types specific to your oroGen components, define them
 * here. Required headers must be included explicitly
 *
 * However, it is common that you will only import types from your library, in
 * which case you do not need this file
 */

namespace lidar_livox {

struct LidarNetInfo {
    uint16_t cmd_data_port;
    uint16_t push_msg_port;
    uint16_t point_data_port;
    uint16_t imu_data_port;
    uint16_t log_data_port;
};

struct HostNetInfo {
    std::string cmd_data_ip;
    uint16_t cmd_data_port;
    std::string push_msg_ip;
    uint16_t push_msg_port;
    std::string point_data_ip;
    uint16_t point_data_port;
    std::string imu_data_ip;
    uint16_t imu_data_port;
    std::string log_data_ip;
    uint16_t log_data_port;
};

}
#endif
