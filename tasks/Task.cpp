/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include "json/json.h"
#include <base-logging/Logging.hpp>
#include <base/Time.hpp>
#include <chrono>
#include <fstream>

using namespace std;
using namespace chrono;
using namespace lidar_livox;

void pointCloudCallback(uint32_t handle,
    const uint8_t dev_type,
    LivoxLidarEthernetPacket* data,
    void* task)
{
    if (data == nullptr) {
        return;
    }
    static_cast<Task*>(task)->processPointcloudData(data);
}

void configurationSetCallback(livox_status status,
    uint32_t handle,
    LivoxLidarAsyncControlResponse* response,
    void* data)
{
    auto& task(*static_cast<Task*>(data));
    if (status != kLivoxLidarStatusSuccess) {
        LOG_ERROR_S << "Setting lidar configuration failed." << endl;
        task.notifyCommandFailure(status);
    }
    if (response == nullptr) {
        return;
    }
    LOG_INFO_S << "configurationSetCallback, status: " << status << ", handle: " << handle
               << ", return code:" << unsigned(response->ret_code)
               << ", key with error: " << unsigned(response->error_key) << endl;

    response->ret_code == LidarReturnCode::SUCCESS
        ? task.notifyCommandSuccess()
        : task.notifyCommandFailure(response->ret_code);
}

void queryInternalInfoCallback(livox_status status,
    uint32_t handle,
    LivoxLidarDiagInternalInfoResponse* response,
    void* client_data)
{
    auto& task(*static_cast<Task*>(client_data));
    if (status != kLivoxLidarStatusSuccess) {
        LOG_ERROR_S << "Setting lidar configuration failed." << endl;
        task.notifyCommandFailure(status);
    }
    if (response == nullptr) {
        return;
    }

    task.proccessInfoData(response);
}

void lidarInfoChangeCallback(const uint32_t handle,
    const LivoxLidarInfo* info,
    void* client_data)
{
    auto& task(*static_cast<Task*>(client_data));
    if (info == nullptr) {
        LOG_ERROR_S << "lidar info change callback failed, the info is nullptr." << endl;
        task.notifyCommandFailure(1);
        return;
    }
    LOG_INFO_S << "LidarInfoChangeCallback Lidar handle:" << handle << " SN: " << info->sn
               << endl;

    task.handle = handle;
    task.notifyCommandSuccess();
}

Task::Task(std::string const& name)
    : TaskBase(name)
{
}

Task::~Task()
{
}

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

    m_command_completed = false;
    SetLivoxLidarInfoChangeCallback(lidarInfoChangeCallback, this);
    waitForCommandSuccess();

    applyCommand(QueryLivoxLidarInternalInfo(handle, queryInternalInfoCallback, this));

    if (!configureLidar()) {
        LOG_ERROR_S << "Failed to configure the lidar!" << std::endl;
        return false;
    }
    LOG_INFO_S << "Finished configuration process!!!" << endl;
    return true;
}

bool Task::startHook()
{
    if (!TaskBase::startHook())
        return false;

    updatePointcloudReceivedStatus(false);

    // Start Motor
    applyCommand(
        SetLivoxLidarWorkMode(handle, kLivoxLidarNormal, configurationSetCallback, this));

    SetLivoxLidarPointCloudCallBack(pointCloudCallback, this);
    waitForPointcloudSuccess();
    return true;
}

void Task::updateHook()
{
    TaskBase::updateHook();
}

void Task::errorHook()
{
    TaskBase::errorHook();
    applyCommand(
        SetLivoxLidarWorkMode(handle, kLivoxLidarWakeUp, configurationSetCallback, this));
}

void Task::stopHook()
{
    TaskBase::stopHook();
    applyCommand(
        SetLivoxLidarWorkMode(handle, kLivoxLidarWakeUp, configurationSetCallback, this));

    SetLivoxLidarPointCloudCallBack(nullptr, this);
}

void Task::cleanupHook()
{
    LivoxLidarSdkUninit();
    TaskBase::cleanupHook();
}

std::string Task::manageJsonFile()
{
    Json::Value data;
    Json::Value lidar;
    Json::Value lidar_net_info;
    Json::Value host_net_info;

    LidarNetInfo lidar_config = _lidar_net_info.get();
    lidar_net_info["cmd_data_port"] = lidar_config.cmd_data_port;
    lidar_net_info["push_msg_port"] = lidar_config.push_msg_port;
    lidar_net_info["point_data_port"] = lidar_config.point_data_port;
    lidar_net_info["imu_data_port"] = lidar_config.imu_data_port;
    lidar_net_info["log_data_port"] = lidar_config.log_data_port;

    HostNetInfo host_config = _host_net_info.get();
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

    lidar["lidar_net_info"] = lidar_net_info;
    lidar["host_net_info"] = host_net_info;

    switch (_lidar_model.get()) {
        case LidarModel::HAP_T1:
            data["HAP"] = lidar;
            break;

        case LidarModel::HAP_TX:
            data["HAP"] = lidar;
            break;

        case LidarModel::MID_360:
            data["MID360"] = lidar;
            break;

        default:
            LOG_ERROR_S << "Device Model configuration not found." << endl;
            return "";
    }

    while (true) {
        auto time = base::Time::now();
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

template <typename F>
void Task::convertPointcloud(LivoxLidarEthernetPacket const* data,
    F const* p_point_data,
    float unit_base)
{
    if (m_point_cloud.time.isNull()) {
        m_point_cloud.time = base::Time::now();
    }

    for (uint32_t i = 0; i < data->dot_num; i++) {
        if (!p_point_data[i].x && !p_point_data[i].y && !p_point_data[i].z) {
            continue;
        }

        m_point_cloud.points.push_back(base::Point(p_point_data[i].x / unit_base,
            p_point_data[i].y / unit_base,
            p_point_data[i].z / unit_base));
        if (_pointcloud_with_color_based_on_reflexivity.get()) {
            m_point_cloud.colors.push_back(
                colorByReflectivity(p_point_data[i].reflectivity));
        }
    }

    if (base::Time::now() - m_point_cloud.time >=
        base::Time::fromSeconds(1.0 / _frequency.get())) {
        _point_cloud.write(m_point_cloud);
        m_point_cloud.time = base::Time::now();
        m_point_cloud.points.clear();
        m_point_cloud.colors.clear();
    }
}

void Task::updatePointcloudReceivedStatus(bool status)
{
    {
        unique_lock<std::mutex> lock(m_pointcloud_sync_mutex);
        m_pointcloud_received = status;
    }
    m_pointcloud_sync_condition.notify_all();
}

void Task::waitForPointcloudSuccess()
{
    unique_lock<std::mutex> lock(m_pointcloud_sync_mutex);
    while (!m_pointcloud_received) {
        // At the moment if something fails, e.g. configuration or timeout and exception
        if (m_pointcloud_sync_condition.wait_for(lock,
                static_cast<std::chrono::milliseconds>(
                    _timeout.get().toMilliseconds())) == std::cv_status::timeout) {
            LOG_ERROR_S << "Still waiting for pointcloud." << endl;
        }
    }
}
void Task::processPointcloudData(LivoxLidarEthernetPacket const* data)
{
    if (m_pointcloud_received == false) {
        updatePointcloudReceivedStatus(true);
    }

    switch (data->data_type) {
        case kLivoxLidarCartesianCoordinateHighData: {
            LivoxLidarCartesianHighRawPoint const* p_point_data =
                reinterpret_cast<LivoxLidarCartesianHighRawPoint const*>(data->data);
            convertPointcloud(data, p_point_data, 1000);
        } break;

        case kLivoxLidarCartesianCoordinateLowData: {
            LivoxLidarCartesianLowRawPoint const* p_point_data =
                reinterpret_cast<LivoxLidarCartesianLowRawPoint const*>(data->data);
            convertPointcloud(data, p_point_data, 100);
        } break;

        case kLivoxLidarSphericalCoordinateData:
            LOG_ERROR_S << "Spherical Coordinate Data not implemented yet." << endl;
            break;

        default:
            LOG_ERROR_S << "Unknown Data Type." << endl;
    }
}

base::Vector4d Task::colorByReflectivity(uint8_t reflectivity)
{
    if (reflectivity < 30) {
        return base::Vector4d(0,
            static_cast<int>(reflectivity * 255 / 30) & 0xff,
            0xff,
            0xff);
    }
    else if (reflectivity < 90) {
        return base::Vector4d(0,
            0xff,
            static_cast<int>((90 - reflectivity) * 255 / 60) & 0xff,
            0xff);
    }
    else if (reflectivity < 150) {
        return base::Vector4d(static_cast<int>((reflectivity - 90) * 255 / 60) & 0xff,
            0xff,
            0,
            0xff);
    }
    else {
        return base::Vector4d(0xff,
            static_cast<int>((255 - reflectivity) * 255 / (256 - 150)) & 0xff,
            0,
            0xff);
    }
}

void Task::notifyCommandSuccess()
{
    unique_lock<std::mutex> lock(m_command_sync_mutex);
    m_command_completed = true;
    m_error_code = 0;
    m_command_sync_condition.notify_all();
}

void Task::notifyCommandFailure(int error_code)
{
    unique_lock<std::mutex> lock(m_command_sync_mutex);
    m_command_completed = true;
    m_error_code = error_code;
    m_command_sync_condition.notify_all();
}

void Task::waitForCommandSuccess()
{
    unique_lock<std::mutex> lock(m_command_sync_mutex);
    while (!m_command_completed) {
        // At the moment if something fails, e.g. configuration or timeout and exception
        // is thrown and the sensor component will quit.
        if (m_command_sync_condition.wait_for(lock,
                static_cast<std::chrono::milliseconds>(
                    _timeout.get().toMilliseconds())) == std::cv_status::timeout) {
            LOG_ERROR_S << "Error code: " << 4 << endl;
            throw std::runtime_error(m_lidar_status.at(4));
        }
    }

    if (m_error_code) {
        try {
            LOG_ERROR_S << "Error code: " << m_error_code << endl;
            throw std::runtime_error(
                m_return_code.at(static_cast<LidarReturnCode>(m_error_code)));
        }
        catch (const std::out_of_range& e) {
            LOG_ERROR_S << "Error code: " << m_error_code << endl;
            throw std::runtime_error("Lidar Unkown error!");
        }
    }
}

void Task::lidarStatusFailure(livox_status status)
{
    try {
        throw std::runtime_error(m_lidar_status.at(status));
    }
    catch (const std::out_of_range& e) {
        throw std::runtime_error("Lidar Unkown error!");
    }
}

template <typename F> void Task::applyCommand(F f)
{
    m_command_completed = false;
    auto status = F();
    status == LidarReturnCode::SUCCESS ? waitForCommandSuccess()
                                       : lidarStatusFailure(status);
}

bool Task::configureLidar()
{
    applyCommand(QueryLivoxLidarInternalInfo(handle, queryInternalInfoCallback, this));

    // This configuration reboots the lidar. It must be first, since all other settings
    // are reset
    auto scan_pattern = _scan_pattern.get();
    if (scan_pattern != m_lidar_state_info.pattern_mode) {
        LOG_INFO_S << "Configuring scan pattern."
                   << " Current: " << m_lidar_state_info.pattern_mode
                   << " desired: " << scan_pattern << endl;
        applyCommand(SetLivoxLidarScanPattern(handle,
            static_cast<LivoxLidarScanPattern>(scan_pattern),
            configurationSetCallback,
            this));
    }

    applyCommand(QueryLivoxLidarInternalInfo(handle, queryInternalInfoCallback, this));
    if (scan_pattern != m_lidar_state_info.pattern_mode) {
        LOG_ERROR_S << " Failed configuring scan pattern!!" << endl;
    }

    auto point_data_type = _point_data_type.get();
    if (point_data_type != m_lidar_state_info.pcl_data_type) {
        LOG_INFO_S << "Configuring pointcloud data type."
                   << " Current: " << m_lidar_state_info.pcl_data_type
                   << " desired: " << point_data_type << endl;

        applyCommand(SetLivoxLidarPclDataType(handle,
            static_cast<LivoxLidarPointDataType>(point_data_type),
            configurationSetCallback,
            this));
    }

    applyCommand(QueryLivoxLidarInternalInfo(handle, queryInternalInfoCallback, this));
    if (point_data_type != m_lidar_state_info.pcl_data_type) {
        LOG_ERROR_S << "Failed configuring pointcloud data type." << endl;
    }

    auto enable_dual_emit = _dual_emit.get();
    LOG_INFO_S << "Enabling dual emit." << endl;
    applyCommand(
        SetLivoxLidarDualEmit(handle, enable_dual_emit, configurationSetCallback, this));
    // No feedback from QueryLivoxLidarInternalInfo

    auto attitude = _install_attitude.get();
    LOG_INFO_S << "Configuring install attitude." << endl;
    LivoxLidarInstallAttitude livox_attitude;
    memcpy(&livox_attitude, &attitude, sizeof(attitude));
    applyCommand(SetLivoxLidarInstallAttitude(handle,
        &livox_attitude,
        configurationSetCallback,
        this));

    applyCommand(QueryLivoxLidarInternalInfo(handle, queryInternalInfoCallback, this));
    if (attitude.roll_deg != m_lidar_state_info.install_attitude.roll_deg ||
        attitude.pitch_deg != m_lidar_state_info.install_attitude.pitch_deg ||
        attitude.yaw_deg != m_lidar_state_info.install_attitude.yaw_deg ||
        attitude.x != m_lidar_state_info.install_attitude.x ||
        attitude.y != m_lidar_state_info.install_attitude.y ||
        attitude.z != m_lidar_state_info.install_attitude.z) {
        LOG_ERROR_S << " Failed configuring Attitude!!" << endl;
    }

    auto blind_spot = _blind_spot.get();
    LOG_INFO_S << "Configuring blind spot. Desired: " << blind_spot << endl;
    applyCommand(SetLivoxLidarBlindSpot(handle,
        static_cast<uint32_t>(blind_spot * 100),
        configurationSetCallback,
        this));
    // No feedback from QueryLivoxLidarInternalInfo

    auto enable_glass_heat = _enable_glass_heat.get();
    if (enable_glass_heat) {
        LOG_INFO_S << "Enabling glass heat." << endl;
        applyCommand(EnableLivoxLidarGlassHeat(handle, configurationSetCallback, this));
        // No feedback from QueryLivoxLidarInternalInfo
    }
    else {
        LOG_INFO_S << "Disabling glass heat." << endl;
        applyCommand(DisableLivoxLidarGlassHeat(handle, configurationSetCallback, this));
        // No feedback from QueryLivoxLidarInternalInfo
    }

    // Only HAP TX supports the IMU function
    if (_lidar_model.get() == LidarModel::HAP_TX) {
        auto enable_imu = _enable_imu.get();
        if (enable_imu && !m_lidar_state_info.imu_data_en) {
            LOG_INFO_S << "Enabling IMU publisher."
                       << " Current: " << m_lidar_state_info.imu_data_en
                       << " desired: " << enable_imu << endl;
            applyCommand(EnableLivoxLidarImuData(handle, configurationSetCallback, this));
        }
        else if (!enable_imu && m_lidar_state_info.imu_data_en) {
            LOG_INFO_S << "Disabling IMU publisher."
                       << " Current: " << m_lidar_state_info.imu_data_en
                       << " desired: " << enable_imu << endl;
            applyCommand(
                DisableLivoxLidarImuData(handle, configurationSetCallback, this));
        }

        if (enable_imu && !m_lidar_state_info.imu_data_en) {
            LOG_INFO_S << "Failed!!!" << endl;
        }
    }

    // // mid360 lidar does not support
    if (_lidar_model.get() != LidarModel::MID_360) {
        auto enable_fusa = _enable_fusa_function.get();
        if (enable_fusa) {
            LOG_INFO_S << "Enable Fusa function." << endl;
            applyCommand(
                EnableLivoxLidarFusaFunciont(handle, configurationSetCallback, this));
        }
        else {
            LOG_INFO_S << "Disable Fusa function." << endl;
            applyCommand(
                DisableLivoxLidarFusaFunciont(handle, configurationSetCallback, this));
        }
        // No feedback from QueryLivoxLidarInternalInfo
    }
    return true;
}

void Task::proccessInfoData(LivoxLidarDiagInternalInfoResponse* response)
{
    uint16_t off = 0;
    for (uint8_t i = 0; i < response->param_num; ++i) {
        LivoxLidarKeyValueParam* kv = (LivoxLidarKeyValueParam*)&response->data[off];
        switch (kv->key) {
            case kKeyPclDataType:
                m_lidar_state_info.pcl_data_type =
                    static_cast<PointDataType>(kv->value[0]);
                break;
            case kKeyPatternMode:
                m_lidar_state_info.pattern_mode = static_cast<ScanPattern>(kv->value[0]);
                break;
            case kKeyLidarIPCfg: {
                std::string ip_text =
                    to_string(kv->value[0]) + "." + to_string(kv->value[1]) + "." +
                    to_string(kv->value[2]) + "." + to_string(kv->value[3]);
                strcpy(m_lidar_state_info.livox_lidar_ip_info.ip_addr, ip_text.c_str());
                ip_text = to_string(kv->value[4]) + "." + to_string(kv->value[5]) + "." +
                          to_string(kv->value[6]) + "." + to_string(kv->value[7]);
                strcpy(m_lidar_state_info.livox_lidar_ip_info.net_mask, ip_text.c_str());
                ip_text = to_string(kv->value[8]) + "." + to_string(kv->value[9]) + "." +
                          to_string(kv->value[10]) + "." + to_string(kv->value[11]);
                strcpy(m_lidar_state_info.livox_lidar_ip_info.gw_addr, ip_text.c_str());
                break;
            }
            case kKeyLidarPointDataHostIPCfg: {
                std::string ip_text =
                    to_string(kv->value[0]) + "." + to_string(kv->value[1]) + "." +
                    to_string(kv->value[2]) + "." + to_string(kv->value[3]);
                strcpy(m_lidar_state_info.host_point_ip_info.host_ip_addr,
                    ip_text.c_str());
                memcpy(&(m_lidar_state_info.host_point_ip_info.host_point_data_port),
                    &(kv->value[4]),
                    sizeof(uint16_t));
                memcpy(&(m_lidar_state_info.host_point_ip_info.lidar_point_data_port),
                    &(kv->value[6]),
                    sizeof(uint16_t));
                break;
            }
            case kKeyLidarImuHostIPCfg: {
                std::string ip_text =
                    to_string(kv->value[0]) + "." + to_string(kv->value[1]) + "." +
                    to_string(kv->value[2]) + "." + to_string(kv->value[3]);
                strcpy(m_lidar_state_info.host_imu_data_ip_info.host_ip_addr,
                    ip_text.c_str());
                memcpy(&(m_lidar_state_info.host_imu_data_ip_info.host_imu_data_port),
                    &(kv->value[4]),
                    sizeof(uint16_t));
                memcpy(&(m_lidar_state_info.host_imu_data_ip_info.lidar_imu_data_port),
                    &(kv->value[6]),
                    sizeof(uint16_t));
                break;
            }
            case kKeyInstallAttitude:
                memcpy(&m_lidar_state_info.install_attitude, &(kv->value[0]), kv->length);
                break;
            case kKeyWorkMode:
                m_lidar_state_info.work_mode = static_cast<LidarWorkMode>(kv->value[0]);
                break;
            case kKeyImuDataEn:
                m_lidar_state_info.imu_data_en = static_cast<bool>(kv->value[0]);
                break;
            case kKeySn:
                memcpy(&m_lidar_state_info.sn, &(kv->value[0]), kv->length);
                break;
            case kKeyProductInfo:
                memcpy(&m_lidar_state_info.product_info, &(kv->value[0]), kv->length);
                break;
            case kKeyVersionApp:
                memcpy(&m_lidar_state_info.version_app, &(kv->value[0]), kv->length);
                break;
            case kKeyVersionLoader:
                memcpy(&m_lidar_state_info.version_load, &(kv->value[0]), kv->length);
                break;
            case kKeyVersionHardware:
                memcpy(&m_lidar_state_info.version_hardware, &(kv->value[0]), kv->length);
                break;
            case kKeyMac:
                memcpy(&m_lidar_state_info.mac, &(kv->value[0]), kv->length);
                break;
        }
        off += sizeof(uint16_t) * 2;
        off += kv->length;
    }
    notifyCommandSuccess();
}