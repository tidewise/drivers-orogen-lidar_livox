/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include "json/json.h"
#include <base-logging/Logging.hpp>
#include <base/Time.hpp>
#include <fstream>

using namespace std;
using namespace lidar_livox;

void PointCloudCallback(uint32_t handle,
    const uint8_t dev_type,
    LivoxLidarEthernetPacket* data,
    void* task)
{
    if (data == nullptr) {
        return;
    }
    static_cast<Task*>(task)->processPointcloudData(data);
}

void LidarInfoChangeCallback(const uint32_t handle,
    const LivoxLidarInfo* info,
    void* task)
{
    if (info == nullptr) {
        printf("lidar info change callback failed, the info is nullptr.\n");
        return;
    }
    printf("LidarInfoChangeCallback Lidar handle: %u SN: %s\n", handle, info->sn);
    SetLivoxLidarWorkMode(handle, kLivoxLidarNormal, nullptr, nullptr);
}

Task::Task(std::string const& name)
    : TaskBase(name)
{
}

Task::~Task()
{
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (!TaskBase::configureHook())
        return false;

    std::string file_name = manageJsonFile();
    if (!LivoxLidarSdkInit(file_name.c_str())) {
        LOG_ERROR_S << "Failed to connect to Lidar!" << std::endl;
        LivoxLidarSdkUninit();
        return false;
    }
    m_measurements_to_merge = _number_of_measurements_per_pointcloud.get();
    SetLivoxLidarInfoChangeCallback(LidarInfoChangeCallback, nullptr);
    return true;
}
bool Task::startHook()
{
    if (!TaskBase::startHook())
        return false;
    SetLivoxLidarPointCloudCallBack(PointCloudCallback, this);
    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();
}
void Task::errorHook()
{
    TaskBase::errorHook();
    LivoxLidarSdkUninit();
}
void Task::stopHook()
{
    TaskBase::stopHook();
    LivoxLidarSdkUninit();
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
    LivoxLidarSdkUninit();
}

std::string Task::manageJsonFile()
{
    Json::Value data;
    Json::Value hap;
    Json::Value lidar_net_info;
    Json::Value host_net_info;

    LidarNetInfo lidar_config = _config_lidar_net_info.get();
    lidar_net_info["cmd_data_port"] = lidar_config.cmd_data_port;
    lidar_net_info["push_msg_port"] = lidar_config.push_msg_port;
    lidar_net_info["point_data_port"] = lidar_config.point_data_port;
    lidar_net_info["imu_data_port"] = lidar_config.imu_data_port;
    lidar_net_info["log_data_port"] = lidar_config.log_data_port;

    HostNetInfo host_config = _config_host_net_info.get();
    host_net_info["cmd_data_ip"] = host_config.cmd_data_ip;
    host_net_info["cmd_data_port"] = host_config.cmd_data_port;
    host_net_info["push_msg_ip"] = host_config.push_msg_ip;
    host_net_info["push_msg_port"] = host_config.push_msg_port;
    host_net_info["point_data_ip"] = host_config.point_data_ip;
    host_net_info["point_data_port"] = host_config.point_data_port;
    host_net_info["imu_data_ip"] = host_config.imu_data_ip;
    host_net_info["imu_data_port"] = host_config.imu_data_port;
    host_net_info["log_data_ip"] = host_config.log_data_ip;
    host_net_info["log_data_port"] = host_config.log_data_port;

    hap["lidar_net_info"] = lidar_net_info;
    hap["host_net_info"] = host_net_info;
    data["HAP"] = hap;

    while (true) {
        base::Time time;
        time.now();
        std::string file_name = "livox-" + getName() + "-" +
                                time.toString(base::Time::Resolution::Milliseconds,
                                    base::Time::DEFAULT_FORMAT) +
                                ".json";
        {
            std::fstream file_id(file_name.c_str(), ios_base::in);
            if (file_id) {
                usleep(10000);
                continue;
            }
        }

        std::fstream file_id(file_name.c_str(), ios_base::out);
        Json::StyledWriter styledWriter;
        file_id << styledWriter.write(data);
        return file_name;
    }
    return "";
}

void Task::processPointcloudData(LivoxLidarEthernetPacket const* data)
{
    if (data->data_type == kLivoxLidarCartesianCoordinateHighData) {
        LivoxLidarCartesianHighRawPoint const* p_point_data =
            reinterpret_cast<LivoxLidarCartesianHighRawPoint const*>(data->data);

        if (m_point_cloud.time.isNull()) {
            m_point_cloud.time = base::Time::now();
        }

        for (uint32_t i = 0; i < data->dot_num; i++) {
            if (!p_point_data[i].x && !p_point_data[i].y && !p_point_data[i].z) {
                continue;
            }

            m_point_cloud.points.push_back(
                base::Point(p_point_data[i].x, p_point_data[i].y, p_point_data[i].z));
        }

        if (++m_measurements_merged == m_measurements_to_merge) {
            _point_cloud.write(m_point_cloud);

            m_point_cloud.time = base::Time();
            m_point_cloud.points.clear();
            m_measurements_merged = 0;
        }
    }
}