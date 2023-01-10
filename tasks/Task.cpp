/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include "json/json.h"
#include <fstream>
#include <base-logging/Logging.hpp>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <thread>
#include <chrono>
#include <iostream>


void PointCloudCallback(uint32_t handle,
    const uint8_t dev_type,
    LivoxLidarEthernetPacket* data,
    void* client_data)
{
    if (data == nullptr) {
        return;
    }
    printf("point cloud handle: %u, data_num: %d, data_type: %d, length: %d, "
           "frame_counter: %d\n",
        handle,
        data->dot_num,
        data->data_type,
        data->length,
        data->frame_cnt);

    if (data->data_type == kLivoxLidarCartesianCoordinateHighData) {
        LivoxLidarCartesianHighRawPoint* p_point_data =
            (LivoxLidarCartesianHighRawPoint*)data->data;
        for (uint32_t i = 0; i < data->dot_num; i++) {
            std::cout << p_point_data[i].x << p_point_data[i].y << p_point_data[i].z
                      << std::endl;
        }
    }
    else if (data->data_type == kLivoxLidarCartesianCoordinateLowData) {
        LivoxLidarCartesianLowRawPoint* p_point_data =
            (LivoxLidarCartesianLowRawPoint*)data->data;
        std::cout << p_point_data[0].x;
    }
    else if (data->data_type == kLivoxLidarSphericalCoordinateData) {
        LivoxLidarSpherPoint* p_point_data = (LivoxLidarSpherPoint*)data->data;
        std::cout << p_point_data[0].theta;
    }
}

using namespace lidar_livox;

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

    manageJsonFile();
    std::string teste = "teste.json";
    if (!LivoxLidarSdkInit(teste.c_str())) {
        LOG_ERROR_S << "Failed to connect to Lidar!" << std::endl;
        LivoxLidarSdkUninit();
        return false;
    }
    SetLivoxLidarPointCloudCallBack(PointCloudCallback, nullptr);
    return true;
}
bool Task::startHook()
{
    if (!TaskBase::startHook())
        return false;
    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();
    _output.write(1);
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


// int main(int argc, const char* argv[])
// {
//     if (argc != 2) {
//         printf("Params Invalid, must input config path.\n");
//         return -1;
//     }
//     const std::string path = argv[1];

//     if (!LivoxLidarSdkInit(path.c_str())) {
//         printf("Livox Init Failed\n");
//         LivoxLidarSdkUninit();
//         return -1;
//     }
//     SetLivoxLidarPointCloudCallBack(PointCloudCallback, nullptr);

//     sleep(300);

//     LivoxLidarSdkUninit();
//     printf("Livox Quick Start Demo End!\n");
//     return 0;
// }

void Task::manageJsonFile()
{
    Json::Value data;
    Json::Value hap;
    Json::Value lidar_net_info;
    Json::Value host_net_info;

    lidar_net_info["cmd_data_port"] = 56000;
    lidar_net_info["push_msg_port"] = 0;
    lidar_net_info["point_data_port"] = 57000;
    lidar_net_info["imu_data_port"] = 58000;
    lidar_net_info["log_data_port"] = 59000;

    host_net_info["cmd_data_ip"] = "192.168.1.50";
    host_net_info["cmd_data_port"] = 56000;
    host_net_info["push_msg_ip"] = "";
    host_net_info["push_msg_port"] = 0;
    host_net_info["point_data_ip"] = "192.168.1.50";
    host_net_info["point_data_port"] = 57000;
    host_net_info["imu_data_ip"] = "192.168.1.50";
    host_net_info["imu_data_port"] = 58000;
    host_net_info["log_data_ip"] = "";
    host_net_info["log_data_port"] = 5900;

    hap["lidar_net_info"] = lidar_net_info;
    hap["host_net_info"] = host_net_info;
    data["HAP"] = hap;

    std::ofstream file_id;
    file_id.open("teste.json");

    Json::StyledWriter styledWriter;
    file_id << styledWriter.write(data);

    file_id.close();
}