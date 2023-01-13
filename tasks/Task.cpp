/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include "json/json.h"
#include <base-logging/Logging.hpp>
#include <base/Time.hpp>
#include <fstream>

using namespace std;
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
    if (response == nullptr) {
        return;
    }
    LOG_INFO_S << "configurationSetCallback, status: " << status << ", handle: " << handle
               << ", ret_code:" << unsigned(response->ret_code)
               << ", error_key: " << unsigned(response->error_key) << endl;

    response->ret_code == 0 ? task.notifyCommandSuccess()
                            : task.notifyCommandFailure(response->ret_code);
}

void queryInternalInfoCallback(livox_status status,
    uint32_t handle,
    LivoxLidarDiagInternalInfoResponse* response,
    void* client_data)
{
    auto& task(*static_cast<Task*>(client_data));
    if (status != kLivoxLidarStatusSuccess) {
        printf("Query lidar internal info failed.\n");
        QueryLivoxLidarInternalInfo(handle, queryInternalInfoCallback, nullptr);
        return;
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
    SetLivoxLidarWorkMode(handle, kLivoxLidarWakeUp, nullptr, nullptr);
    LivoxLidarSdkUninit();
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
    m_measurements_to_merge = _number_of_measurements_per_pointcloud.get();
    SetLivoxLidarInfoChangeCallback(lidarInfoChangeCallback, this);
    waitForCommandSuccess();
    QueryLivoxLidarInternalInfo(handle, queryInternalInfoCallback, this);
    waitForCommandSuccess();
    if (!configureLidar()) {
        LOG_ERROR_S << "Failed to configure the lidar!" << std::endl;
        return false;
    }
    return true;
}

bool Task::startHook()
{
    if (!TaskBase::startHook())
        return false;
    SetLivoxLidarPointCloudCallBack(pointCloudCallback, this);
    return true;
}

void Task::updateHook()
{
    TaskBase::updateHook();
}

void Task::errorHook()
{
    TaskBase::errorHook();
    SetLivoxLidarWorkMode(handle, kLivoxLidarWakeUp, configurationSetCallback, nullptr);
}

void Task::stopHook()
{
    TaskBase::stopHook();
    SetLivoxLidarWorkMode(handle, kLivoxLidarWakeUp, configurationSetCallback, nullptr);
}

void Task::cleanupHook()
{
    TaskBase::cleanupHook();
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

            m_point_cloud.points.push_back(base::Point(p_point_data[i].x / 1000.0,
                p_point_data[i].y / 1000.0,
                p_point_data[i].z / 1000.0));
        }

        if (++m_measurements_merged == m_measurements_to_merge) {
            _point_cloud.write(m_point_cloud);
            _lidar_status.write(m_lidar_state_info);
            m_point_cloud.time = base::Time();
            m_point_cloud.points.clear();
            m_measurements_merged = 0;
        }
    }

    // TODO CARTESIANCOORDINATELOWDATA
    // TODO SPHERICALCOORDINATEDATA
}

void Task::notifyCommandSuccess()
{
    unique_lock<std::mutex> lock(m_command_sync_mutex);
    m_error_code = 0;
    m_command_sync_condition.notify_all();
}

void Task::notifyCommandFailure(int error_code)
{
    unique_lock<std::mutex> lock(m_command_sync_mutex);
    m_error_code = error_code;
    m_command_sync_condition.notify_all();
}

void Task::waitForCommandSuccess()
{
    unique_lock<std::mutex> lock(m_command_sync_mutex);

    m_command_sync_condition.wait(lock);
    if (m_error_code) {
        try {
            throw std::runtime_error(m_lidar_status.at(m_error_code));
        }
        catch (const std::out_of_range& e) {
            throw std::runtime_error("Unkown error!");
        }
    }
}

bool Task::configureLidar()
{
    SetLivoxLidarPclDataType(handle,
        static_cast<LivoxLidarPointDataType>(_conf_point_data_type.get()),
        configurationSetCallback,
        this);
    waitForCommandSuccess();

    // SetLivoxLidarScanPattern(handle,
    //     static_cast<LivoxLidarScanPattern>(_conf_scan_pattern.get()),
    //     configurationSetCallback,
    //     this);
    // waitForCommandSuccess();

    // SetLivoxLidarDualEmit(handle, _conf_dual_emit.get(), configurationSetCallback,
    // this); waitForCommandSuccess();

    // if (_conf_send_point_cloud.get()) {
    //     EnableLivoxLidarPointSend(handle, configurationSetCallback, this);
    // }
    // else {
    //     DisableLivoxLidarPointSend(handle, configurationSetCallback, this);
    // }
    // waitForCommandSuccess();

    // LidarInstallAttitude attitude = _conf_install_attitude.get();
    // LivoxLidarInstallAttitude livox_attitude;
    // memcpy(&livox_attitude, &attitude, sizeof(attitude));
    // SetLivoxLidarInstallAttitude(handle, &livox_attitude, configurationSetCallback,
    // this);

    // // Still need to check
    // {
    //     /**
    //      * Set LiDAR Ip info.
    //      * @param  handle        device handle.
    //      * @param  ipconfig      lidar ip info.
    //      * @param  cb            callback for the command.
    //      * @param  client_data   user data associated with the command.
    //      * @return kStatusSuccess on successful return, see \ref LivoxStatus for other
    //      * error code.
    //      */
    //     livox_status SetLivoxLidarIp(uint32_t handle,
    //         LivoxLidarIpInfo * ip_config,
    //         LivoxLidarAsyncControlCallback cb,
    //         void* client_data);

    //     /**
    //      * Set LiDAR state Ip info.
    //      * @param  handle                 device handle.
    //      * @param  host_state_info_ipcfg  lidar ip info.
    //      * @param  cb                     callback for the command.
    //      * @param  client_data            user data associated with the command.
    //      * @return kStatusSuccess on successful return, see \ref LivoxStatus for other
    //      * error code.
    //      */
    //     livox_status SetLivoxLidarStateInfoHostIPCfg(uint32_t handle,
    //         HostStateInfoIpInfo * host_state_info_ipcfg,
    //         LivoxLidarAsyncControlCallback cb,
    //         void* client_data);

    //     /**
    //      * Set LiDAR point cloud host ip info.
    //      * @param  handle                 device handle.
    //      * @param  host_point_ipcfg       lidar ip info.
    //      * @param  cb                     callback for the command.
    //      * @param  client_data            user data associated with the command.
    //      * @return kStatusSuccess on successful return, see \ref LivoxStatus for other
    //      * error code.
    //      */
    //     livox_status SetLivoxLidarPointDataHostIPCfg(uint32_t handle,
    //         HostPointIPInfo * host_point_ipcfg,
    //         LivoxLidarAsyncControlCallback cb,
    //         void* client_data);

    //     /**
    //      * Set LiDAR imu data host ip info.
    //      * @param  handle                 device handle.
    //      * @param  host_imu_ipcfg         lidar ip info.
    //      * @param  cb                     callback for the command.
    //      * @param  client_data            user data associated with the command.
    //      * @return kStatusSuccess on successful return, see \ref LivoxStatus for other
    //      * error code.
    //      */
    //     livox_status SetLivoxLidarImuDataHostIPCfg(uint32_t handle,
    //         HostImuDataIPInfo * host_imu_ipcfg,
    //         LivoxLidarAsyncControlCallback cb,
    //         void* client_data);
    // }
    // // end
    // // auto coiso2 = static_cast<FovCfg>(_conf_fov_cfg0.get());
    // // SetLivoxLidarFovCfg0(handle, coiso2, configurationSetCallback, this);
    // // waitForCommandSuccess();
    // // auto coiso1 = static_cast<FovCfg>(_conf_fov_cfg1.get());
    // // SetLivoxLidarFovCfg1(handle, coiso1, configurationSetCallback, this);
    // // waitForCommandSuccess();

    // // /**
    // //  * Enable LiDAR fov cfg1.
    // //  * @param  handle          device handle.
    // //  * @param  cb              callback for the command.
    // //  * @param  client_data     user data associated with the command.
    // //  * @return kStatusSuccess on successful return, see \ref LivoxStatus for other
    // //  error
    // //  * code.
    // //  */
    // // EnableLivoxLidarFov(handle, uint8_t fov_en, configurationSetCallback, this);
    // // waitForCommandSuccess();

    // // /**
    // //  * Disable LiDAR fov.
    // //  * @param  handle          device handle.
    // //  * @param  cb              callback for the command.
    // //  * @param  client_data     user data associated with the command.
    // //  * @return kStatusSuccess on successful return, see \ref LivoxStatus for other
    // //  error
    // //  * code.
    // //  */
    // // DisableLivoxLidarFov(handle, configurationSetCallback, this);
    // // waitForCommandSuccess();

    // SetLivoxLidarDetectMode(handle,
    //     static_cast<LivoxLidarDetectMode>(_conf_lidar_detect_mode.get()),
    //     configurationSetCallback,
    //     this);
    // waitForCommandSuccess();

    // // /**
    // //  * Set LiDAR func io cfg.
    // //  * @param  handle          device handle.
    // //  * @param  func_io_cfg     func io cfg; 0:IN0,1:IN1, 2:OUT0, 3:OUT1;Mid360 lidar
    // 8,
    // //  * 10, 12, 11
    // //  * @param  cb              callback for the command.
    // //  * @param  client_data     user data associated with the command.
    // //  * @return kStatusSuccess on successful return, see \ref LivoxStatus for other
    // //  error
    // //  * code.
    // //  */
    // // auto coiso = static_cast<FuncIOCfg>(_conf_function_io.get());
    // // SetLivoxLidarFuncIOCfg(handle, coiso, configurationSetCallback, this);
    // // waitForCommandSuccess();

    // SetLivoxLidarBlindSpot(handle,
    //     _conf_blind_spot.get(),
    //     configurationSetCallback,
    //     this);
    // waitForCommandSuccess();

    // if (_conf_enable_glass_heat.get()) {

    //     EnableLivoxLidarGlassHeat(handle, configurationSetCallback, this);
    // }
    // else {
    //     DisableLivoxLidarGlassHeat(handle, configurationSetCallback, this);
    // }
    // waitForCommandSuccess();

    // SetLivoxLidarGlassHeat(handle,
    //     static_cast<LivoxLidarGlassHeat>(_conf_glass_heat.get()),
    //     configurationSetCallback,
    //     this);
    // waitForCommandSuccess();

    // if (_conf_enable_imu.get()) {

    //     EnableLivoxLidarImuData(handle, configurationSetCallback, this);
    // }
    // else {
    //     DisableLivoxLidarImuData(handle, configurationSetCallback, this);
    // }
    // waitForCommandSuccess();

    // // mid360 lidar does not support
    // if (_conf_enable_fusa_function.get()) {

    //     EnableLivoxLidarFusaFunciont(handle, configurationSetCallback, this);
    // }
    // else {
    //     DisableLivoxLidarFusaFunciont(handle, configurationSetCallback, this);
    // }
    // waitForCommandSuccess();

    SetLivoxLidarWorkMode(handle, kLivoxLidarNormal, configurationSetCallback, this);
    waitForCommandSuccess();
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
            case kKeyDualEmitEnable:
                m_lidar_state_info.dual_emit_en = static_cast<bool>(kv->value[0]);
                break;
            case kKeyPointSendEnable:
                m_lidar_state_info.point_send_en = static_cast<bool>(kv->value[0]);
                break;
                // case kKeyLidarIPCfg:
                //     HostStateInfoIpInfo lidar_data_ip_info =
                //         static_cast<HostStateInfoIpInfo>(kv->value[0]);
                //     break;
                // case (kKeyStateInfoHostIPCfg:
                //     HostPointIPInfo lidar_data_host_info =
                //         static_cast<HostPointIPInfo>(kv->value[0]);
                //     break;
                // case (kKeyLidarPointDataHostIPCfg:
                //     memcpy(m_lidar_state_info.host_point_ip_info.host_ip_addr,
                //         &(kv->value[0]),
                //         sizeof(uint8_t) * 4);
                //
                // memcpy(&(m_lidar_state_info.host_point_ip_info.host_point_data_port),
                // //         &(kv->value[4]),
                // //         sizeof(uint16_t));
                // //
                // memcpy(&(m_lidar_state_info.host_point_ip_info.lidar_point_data_port),
                // //         &(kv->value[6]),
                // //         sizeof(uint16_t));
                // //     break;
                // // case (kKeyLidarImuHostIPCfg:
                // // memcpy(m_lidar_state_info.host_imu_data_ip_info.host_ip_addr,
                // //     &(kv->value[0]), sizeof(uint8_t) * 4);
                // //
                // memcpy(&(m_lidar_state_info.host_imu_data_ip_info.host_imu_data_port),
                // //     &(kv->value[4]), sizeof(uint16_t));
                // //
                // memcpy(&(m_lidar_state_info.host_imu_data_ip_info.lidar_imu_data_port),
                // //     &(kv->value[6]), sizeof(uint16_t)); break;
                // // case (kKeyInstallAttitude:
                // //     memcpy(&m_lidar_state_info.install_attitude,
                // &(kv->value[0]),
                // //     kv->length); break;
            case kKeyBlindSpotSet:
                m_lidar_state_info.blind_spot_set = static_cast<uint32_t>(kv->value[0]);
                break;
            case kKeyFovCfg0:
                memcpy(&m_lidar_state_info.fov_cfg0, &(kv->value[0]), kv->length);
                break;
            case kKeyFovCfg1:
                memcpy(&m_lidar_state_info.fov_cfg1, &(kv->value[0]), kv->length);
                break;
                // case (kKeyRoiEn:
                //     FovCfg lidar_data_fov_cfg1 =
                // static_cast<FovCfg>(kv->value[0]);
            //     break;
            case kKeyDetectMode:
                m_lidar_state_info.detect_mode =
                    static_cast<LidarDetectMode>(kv->value[0]);
                break;
            case kKeyFuncIOCfg:
                memcpy(&m_lidar_state_info.func_io_cfg, &(kv->value[0]), kv->length);
                break;
            case kKeyWorkMode:
                m_lidar_state_info.work_mode = static_cast<LidarWorkMode>(kv->value[0]);
                break;
            case kKeyGlassHeat:
                m_lidar_state_info.glass_heat = static_cast<LidarGlassHeat>(kv->value[0]);
                break;
            case kKeyImuDataEn:
                m_lidar_state_info.imu_data_en = static_cast<bool>(kv->value[0]);
                break;
            case kKeyFusaEn:
                m_lidar_state_info.fusa_en = static_cast<bool>(kv->value[0]);
                break;
            // case (kKeyLogParamSet:
            //     ? ? ? ? ? ? / break;
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
            case kKeyCurWorkState:
                memcpy(&m_lidar_state_info.cur_work_state, &(kv->value[0]), kv->length);
                break;
            case kKeyCoreTemp:
                memcpy(&m_lidar_state_info.core_temp, &(kv->value[0]), kv->length);
                break;
            case kKeyPowerUpCnt:
                memcpy(&m_lidar_state_info.power_up_cnt, &(kv->value[0]), kv->length);
                break;
            case kKeyLocalTimeNow:
                memcpy(&m_lidar_state_info.local_time_now, &(kv->value[0]), kv->length);
                break;
            case kKeyLastSyncTime:
                memcpy(&m_lidar_state_info.last_sync_time, &(kv->value[0]), kv->length);
                break;
            case kKeyTimeOffset:
                memcpy(&m_lidar_state_info.time_offset, &(kv->value[0]), kv->length);
                break;
            case kKeyTimeSyncType:
                memcpy(&m_lidar_state_info.time_sync_type, &(kv->value[0]), kv->length);
                break;
            // case (kKeyStatusCode:
            //     ?????
            //     break;
            case kKeyLidarDiagStatus:
                memcpy(&m_lidar_state_info.diag_status, &(kv->value[0]), kv->length);
                break;
                // case (kKeyLidarFlashStatus:
                //     FovCfg lidar_data_fov_cfg1 =
                // static_cast<FovCfg>(kv->value[0]);
                //     break;
                // case kKeyHmsCode:
                //     memcpy(&m_lidar_state_info.hms_code, &(kv->value[0]), kv->length);
                //     break;
                // case kKeyFwType:
                //     memcpy(&m_lidar_state_info.fw_type, &(kv->value[0]), kv->length);
                //     break;

                // case (kKeyLidarDiagInfoQue:
                //     break;
        }
        off += sizeof(uint16_t) * 2;
        off += kv->length;
    }
    notifyCommandSuccess();
}