# frozen_string_literal: true

using_task_library "lidar_livox"
import_types_from "lidar_livox"
import_types_from "base"

describe OroGen.lidar_livox.Task do
    run_live

    it "starts and outputs point clouds" do
        task = create_configure_and_start_task

        # The component's startHook should wait for a sample to be received. We don't
        # really have a way to make sure the event is sent after the point cloud (async !)
        # so do a poor man's check by verifying that we get another point cloud rapidly
        output = expect_execution.timeout(0.5).to do
            have_one_new_sample(task.point_cloud_port)
        end


        pp output
        STDIN.readline
        puts "Does the output looks OK ?"
        ask_ok
    end

    it "stops the motor on stop" do
        task = create_configure_and_start_task
        expect_execution { task.stop! }
            .to { emit task.interrupt_event }

        puts "Is the motor disabled ?"
        ask_ok
    end

    it "works when restarted" do
        task = create_configure_and_start_task
        expect_execution { task.stop! }.to { emit task.stop_event }

        task = create_configure_and_start_task
        output = expect_execution.to do
            have_one_new_sample(task.point_cloud_port)
        end

        pp output
        puts "Does the output looks OK ?"
        ask_ok
    end

    it "works when reconfigured" do
        task = create_configure_and_start_task
        expect_execution { task.stop! }.to { emit task.stop_event }
        task.needs_reconfiguration!

        task = create_configure_and_start_task
        output = expect_execution.to do
            have_one_new_sample(task.point_cloud_port)
        end

        pp output
        puts "Does the output looks OK ?"
        ask_ok
    end

    def ask_ok
        puts "OK ? (y/n)"
        value = STDIN.readline.chomp
        raise "test failed" unless value == "y"
    end


    def create_task
        task = syskit_deploy(
            OroGen.lidar_livox
                  .Task
                  .deployed_as("lidar_livox_task")
        )
        task.properties.lidar_net_info = Types.lidar_livox.LidarNetInfo.new
        task.properties.lidar_net_info.cmd_data_port = 56_000
        task.properties.lidar_net_info.push_msg_port = 0
        task.properties.lidar_net_info.point_data_port = 57_000
        task.properties.lidar_net_info.imu_data_port = 58_000
        task.properties.lidar_net_info.log_data_port = 59_000

        task.properties.host_net_info = Types.lidar_livox.HostNetInfo.new
        task.properties.host_net_info.cmd_data_ip = "192.168.1.50"
        task.properties.host_net_info.cmd_data_port = 56_000
        task.properties.host_net_info.push_msg_ip = ""
        task.properties.host_net_info.push_msg_port = 0
        task.properties.host_net_info.point_data_ip = "192.168.1.50"
        task.properties.host_net_info.point_data_port = 57_000
        task.properties.host_net_info.imu_data_ip = "192.168.1.50"
        task.properties.host_net_info.imu_data_port = 58_000
        task.properties.host_net_info.log_data_ip = ""
        task.properties.host_net_info.log_data_port = 5_900

        task.properties.lidar_model = "HAP_T1"
        task.properties.frequency = 10
        task.properties.pointcloud_with_color_based_on_reflexivity = true
        task.properties.timeout = Time.at(20)
        task.properties.point_data_type = Types.lidar_livox.PointDataType.new
        task.properties.point_data_type = "CARTESIAN_COORDINATE_HIGH_DATA"

        task.properties.scan_pattern = "NON_REPETITIVE"
        task.properties.install_attitude = Types.lidar_livox.LidarInstallAttitude.new
        task.properties.install_attitude.roll_deg = 1.0
        task.properties.install_attitude.pitch_deg = 2.0
        task.properties.install_attitude.yaw_deg = 1.0
        task.properties.install_attitude.x = 2.0
        task.properties.install_attitude.y = 1.0
        task.properties.install_attitude.z = 1.0
        task.properties.blind_spot = 0.5
        task.properties.enable_glass_heat = false
        task.properties.enable_imu = true
        task.properties.enable_fusa_function = false

        task
    end

    def create_configure_and_start_task
        task = create_task
        syskit_configure(task)
        expect_execution { task.start! }.timeout(15).to do
            emit task.start_event
        end
        task
    end
end
