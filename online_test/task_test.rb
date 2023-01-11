# frozen_string_literal: true

using_task_library "lidar_livox"
import_types_from "lidar_livox"
import_types_from "base"

describe OroGen.lidar_livox.Task do
    run_live

    attr_reader :task

    before do
        @task = syskit_deploy(
            OroGen.lidar_livox
                  .Task
                  .deployed_as("lidar_livox_task")
        )
        @task.properties.config_lidar_net_info = Types.lidar_livox.LidarNetInfo.new
        @task.properties.config_lidar_net_info.cmd_data_port = 56_000
        @task.properties.config_lidar_net_info.push_msg_port = 0
        @task.properties.config_lidar_net_info.point_data_port = 57_000
        @task.properties.config_lidar_net_info.imu_data_port = 58_000
        @task.properties.config_lidar_net_info.log_data_port = 59_000

        @task.properties.config_host_net_info = Types.lidar_livox.HostNetInfo.new
        @task.properties.config_host_net_info.cmd_data_ip = "192.168.1.50"
        @task.properties.config_host_net_info.cmd_data_port = 56_000
        @task.properties.config_host_net_info.push_msg_ip = ""
        @task.properties.config_host_net_info.push_msg_port = 0
        @task.properties.config_host_net_info.point_data_ip = "192.168.1.50"
        @task.properties.config_host_net_info.point_data_port = 57_000
        @task.properties.config_host_net_info.imu_data_ip = "192.168.1.50"
        @task.properties.config_host_net_info.imu_data_port = 58_000
        @task.properties.config_host_net_info.log_data_ip = ""
        @task.properties.config_host_net_info.log_data_port = 5_900

        @task.properties.number_of_measurements_per_pointcloud = 100
    end

    it "displays information" do
        syskit_configure_and_start(@task)
        output = expect_execution do
        end.to { have_one_new_sample task.point_cloud_port}
        pp "resultado"
        pp output
    end
end
