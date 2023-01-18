/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef LIDAR_LIVOX_TASK_TASK_HPP
#define LIDAR_LIVOX_TASK_TASK_HPP

#include "lidar_livox/TaskBase.hpp"
#include "livox_lidar_api.h"
#include "livox_lidar_def.h"
#include <base/Point.hpp>
#include <base/samples/Pointcloud.hpp>
#include <condition_variable>
#include <map>
#include <mutex>

namespace lidar_livox {

    /*! \class Task
     * \brief The task context provides and requires services. It uses an ExecutionEngine
to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These
interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the
associated workflow.
     * Declare a new task context (i.e., a component)

The corresponding C++ class can be edited in tasks/Task.hpp and
tasks/Task.cpp, and will be put in the lidar_livox namespace.
     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','lidar_livox::Task')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix
argument.
     */
    class Task : public TaskBase {
        friend class TaskBase;

    private:
        std::mutex m_command_sync_mutex;
        std::condition_variable m_command_sync_condition;
        bool m_command_completed = false;

        std::mutex m_pointcloud_sync_mutex;
        std::condition_variable m_pointcloud_sync_condition;
        bool m_pointcloud_received = false;

        int m_error_code = 0;
        int m_measurements_to_merge = 0;
        int m_measurements_merged = 0;
        base::samples::Pointcloud m_point_cloud;

        template <typename F> void applyCommand(F f);

        template <typename F>
        void convertPointcloud(LivoxLidarEthernetPacket const* data,
            F const* point_data,
            float base);

        const std::array<std::string, 10> m_lidar_status = {
            "Success.",
            "Failure.",
            "Requested device is not connected.",
            "Operation is not supported on this device.",
            "Operation timeouts.",
            "No enough memory.",
            "Command channel not exist.",
            "Device handle invalid.",
            "Handler implementation not exist.",
            "Command send failed.",
        };

        const std::map<LidarReturnCode, std::string> m_return_code{
            {                LidarReturnCode::SUCCESS,"Execution succeed"                                                      },
            {                LidarReturnCode::FAILURE,               "Execution failed"},
            {         LidarReturnCode::NOT_PERMIT_NOW, "Current state does not support"},
            {           LidarReturnCode::OUT_OF_RANGE,     "Setting value out of range"},
            {       LidarReturnCode::PARAM_NOTSUPPORT, "The parameter is not supported"},
            {    LidarReturnCode::PARAM_REBOOT_EFFECT,
             "Parameters need to reboot to take effect"                                },
            {          LidarReturnCode::PARAM_RD_ONLY,
             "The parameter is read-only and cannot be written"                        },
            {      LidarReturnCode::PARAM_INVALID_LEN,
             "The request parameter length is wrong, or the ack packet exceeds the "
             "maximum length"                                                          },
            {      LidarReturnCode::PARAM_KEY_NUM_ERR,
             "Parameter key_ num and key_ list mismatch"                               },
            {  LidarReturnCode::UPGRADE_PUB_KEY_ERROR,
             "Public key signature verification error"                                 },
            {   LidarReturnCode::UPGRADE_DIGEST_ERROR,             "Digest check error"},
            {  LidarReturnCode::UPGRADE_FW_TYPE_ERROR,         "Firmware type mismatch"},
            {LidarReturnCode::UPGRADE_FW_OUT_OF_RANGE,   "Firmware length out of range"}
        };

        LidarStateInfo m_lidar_state_info;

    public:
        /** TaskContext constructor for Task
         * \param name Name of the task. This name needs to be unique to make it
         * identifiable via nameservices. \param initial_state The initial TaskState
         * of the TaskContext. Default is Stopped state.
         */
        Task(std::string const& name = "lidar_livox::Task");

        /** Default deconstructor of Task
         */
        ~Task();

        /** This hook is called by Orocos when the state machine transitions
         * from PreOperational to Stopped. If it returns false, then the
         * component will stay in PreOperational. Otherwise, it goes into
         * Stopped.
         *
         * It is meaningful only if the #needs_configuration has been specified
         * in the task context definition with (for example):
         \verbatim
         task_context "TaskName" do
           needs_configuration
           ...
         end
         \endverbatim
         */
        bool configureHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to Running. If it returns false, then the component will
         * stay in Stopped. Otherwise, it goes into Running and updateHook()
         * will be called.
         */
        bool startHook();

        /** This hook is called by Orocos when the component is in the Running
         * state, at each activity step. Here, the activity gives the "ticks"
         * when the hook should be called.
         *
         * The error(), exception() and fatal() calls, when called in this hook,
         * allow to get into the associated RunTimeError, Exception and
         * FatalError states.
         *
         * In the first case, updateHook() is still called, and recover() allows
         * you to go back into the Running state.  In the second case, the
         * errorHook() will be called instead of updateHook(). In Exception, the
         * component is stopped and recover() needs to be called before starting
         * it again. Finally, FatalError cannot be recovered.
         */
        void updateHook();

        /** This hook is called by Orocos when the component is in the
         * RunTimeError state, at each activity step. See the discussion in
         * updateHook() about triggering options.
         *
         * Call recover() to go back in the Runtime state.
         */
        void errorHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Running to Stopped after stop() has been called.
         */
        void stopHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to PreOperational, requiring the call to configureHook()
         * before calling start() again.
         */
        void cleanupHook();

        // Handle used to communicate with lidar
        uint32_t handle;

        /** This function is used to create a json file from orogen configurations
         *  The returned object will be the path of the created json file
         */
        std::string manageJsonFile();

        /** This function converts the received pointcloud data into a syskit point cloud
         * data and publishes it.
         */
        void processPointcloudData(LivoxLidarEthernetPacket const* data);

        /** This function configures the lidar with orogen configuration
         */
        bool configureLidar();

        void notifyCommandSuccess();
        void waitForCommandSuccess();
        void handleCommandReturnValue(livox_status);
        void notifyCommandFailure(int error_code);
        void lidarStatusFailure(livox_status);

        void proccessInfoData(LivoxLidarDiagInternalInfoResponse* response);

        /** Provides a colorscale based on reflectivity
         */
        base::Vector4d colorByReflectivity(uint8_t reflectivity);

        void updatePointcloudReceivedStatus(bool status);
        void waitForPointcloudSuccess();
    };
}

#endif
